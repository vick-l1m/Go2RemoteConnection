"""
gateway_app.py

Go2 Remote Connection — GATEWAY.

This runs on an always-on "front door" machine (NOT on the Go2). The phone/browser
talks ONLY to this gateway; the gateway proxies HTTP + WebSockets through to the Go2's
robot API over the private tailnet. The Go2 is therefore never exposed to the internet.

Topology:

    phone (Tailscale app)
        |  HTTPS, tailnet-only, authenticated by Tailscale
        v
    tailscale serve  --(injects Tailscale-User-* headers)-->  this gateway (127.0.0.1:8000)
        |  tailnet (private)
        v
    Go2 robot FastAPI :8000  (tailnet-only)

Authentication is two layered, and NO secret ever lives in the browser:

  1. Browser  <-> gateway : Tailscale identity. `tailscale serve` injects a trusted
     `Tailscale-User-Login` header (a public Funnel visitor never gets one). The gateway
     binds to localhost only, so the ONLY way in is through `tailscale serve` — which is
     what makes the header unspoofable. The login is matched against a role allowlist
     (viewer / driver / admin), and each request is authorized by role.

  2. Gateway   <-> Go2    : a server-side shared token (GO2_TOKEN), added to proxied
     requests. The Go2 runs with GO2_AUTH_ENABLED=1 and the matching ~/go2_token.

Author: Victor Lim
"""

import asyncio
import json
import logging
import os
import time
from pathlib import Path
from urllib.parse import urlencode

import httpx
import websockets
from fastapi import FastAPI, Request, WebSocket
from fastapi.responses import FileResponse, JSONResponse, PlainTextResponse, Response
from fastapi.staticfiles import StaticFiles
from starlette.datastructures import Headers

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
REPO_ROOT = Path(__file__).resolve().parent.parent
GATEWAY_DIR = Path(__file__).resolve().parent

# The Go2's robot API, reachable only over the tailnet.
GO2_UPSTREAM = os.getenv(
    "GO2_UPSTREAM", "http://unitree-jetson-payload-1.tail85e7d7.ts.net:8000"
).rstrip("/")

# Shared token for the gateway -> Go2 hop. Server-side only: it is added to
# proxied requests here and never reaches the browser.
GO2_TOKEN = (os.getenv("GO2_TOKEN") or "").strip()

# email -> role mapping.
ALLOWLIST_FILE = os.getenv("GATEWAY_ALLOWLIST", str(GATEWAY_DIR / "allowlist.json"))

# The robot UI's static assets are served straight from the repo checkout.
ROBOT_STATIC = Path(
    os.getenv("GATEWAY_STATIC_DIR", str(REPO_ROOT / "src/go2_remote_connection/app/static"))
)
PAGES_DIR = ROBOT_STATIC / "pages"
GATEWAY_PAGES = GATEWAY_DIR / "static"

# Local-testing escape hatch: pretend this login was injected by Tailscale.
# NEVER set this in production — it bypasses the identity check entirely.
DEV_USER = (os.getenv("GATEWAY_DEV_USER") or "").strip()

AUDIT_LOG = os.getenv("GATEWAY_AUDIT_LOG", "/tmp/go2_gateway_audit.log")

# Token-bucket rate limit on mutating requests, per user.
RL_CAPACITY = float(os.getenv("GATEWAY_RL_CAPACITY", "120"))
RL_REFILL = float(os.getenv("GATEWAY_RL_REFILL_PER_SEC", "60"))

UPSTREAM_TIMEOUT = float(os.getenv("GATEWAY_UPSTREAM_TIMEOUT", "10"))

# WebSocket upstream is the same host with the ws/wss scheme.
WS_UPSTREAM = ("wss://" + GO2_UPSTREAM[len("https://"):]) if GO2_UPSTREAM.startswith("https://") \
    else ("ws://" + GO2_UPSTREAM[len("http://"):]) if GO2_UPSTREAM.startswith("http://") \
    else GO2_UPSTREAM

ROLE_RANK = {"viewer": 1, "driver": 2, "admin": 3}

# Headers that must not be forwarded verbatim through a proxy hop.
HOP_BY_HOP = {"connection", "keep-alive", "proxy-authenticate", "proxy-authorization",
              "te", "trailers", "transfer-encoding", "upgrade", "content-length", "host"}


# ----------------------------------------------------------------------------
# Logging + audit trail. Every authorization decision and every proxied
# request is recorded as one JSON line: who drove the robot, and when.
# ----------------------------------------------------------------------------
logging.basicConfig(level=logging.INFO, format="%(asctime)s [gateway] %(message)s")
log = logging.getLogger("go2gateway")

_audit_logger = logging.getLogger("go2gateway.audit")
_audit_logger.setLevel(logging.INFO)
try:
    _fh = logging.FileHandler(AUDIT_LOG)
    _fh.setFormatter(logging.Formatter("%(message)s"))
    _audit_logger.addHandler(_fh)
except OSError as exc:
    log.warning("Could not open audit log %s: %s", AUDIT_LOG, exc)


def audit(user, role, method, path, status, detail=""):
    rec = {
        "ts": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "user": user,
        "role": role,
        "method": method,
        "path": path,
        "status": status,
    }
    if detail:
        rec["detail"] = detail
    line = json.dumps(rec)
    _audit_logger.info(line)
    log.info("AUDIT %s", line)


# ----------------------------------------------------------------------------
# Allowlist + role model
# ----------------------------------------------------------------------------
def load_allowlist():
    """Returns (roles: dict[email->role], default_role: str|None)."""
    try:
        data = json.loads(Path(ALLOWLIST_FILE).read_text(encoding="utf-8"))
    except FileNotFoundError:
        log.warning("Allowlist %s not found — ALL users will be denied. "
                    "Copy allowlist.example.json to allowlist.json and add yourself.",
                    ALLOWLIST_FILE)
        return {}, None
    except (OSError, json.JSONDecodeError) as exc:
        log.error("Failed to read allowlist %s: %s — denying all.", ALLOWLIST_FILE, exc)
        return {}, None

    roles = {k.lower(): v for k, v in (data.get("roles") or {}).items()}
    default_role = data.get("default_role")
    return roles, default_role


ALLOWLIST, DEFAULT_ROLE = load_allowlist()


def role_for(login: str):
    if not login:
        return None
    return ALLOWLIST.get(login.lower(), DEFAULT_ROLE)


def required_role(method: str | None, path: str) -> str:
    """Minimum role needed to touch (method, path)."""
    p = path.rstrip("/") or "/"

    # Anything that can command the robot or open a shell is admin-only.
    if p in ("/ws/terminal", "/terminal"):
        return "admin"
    if p.startswith("/control_mode") and method == "POST":
        return "admin"
    # Swapping the RL policy changes what drives the motors, so it is admin-only.
    # The robot API exposes this as POST /rl/policy/{policy_id} (see
    # src/go2_remote_connection/app/api/rl_policy.py); GET /rl/policies and
    # GET /rl/policy are reads and fall through to viewer.
    if p.startswith("/rl/policy") and method == "POST":
        return "admin"
    if p.startswith("/safety/") and method == "POST":
        return "admin"

    # Any other mutating request: driver.
    if method in ("POST", "PUT", "DELETE", "PATCH"):
        return "driver"

    # Reads are open to anyone on the allowlist.
    return "viewer"


# ----------------------------------------------------------------------------
# Rate limiting
# ----------------------------------------------------------------------------
class RateLimiter:
    def __init__(self, capacity: float, refill_per_sec: float):
        self.cap = capacity
        self.refill = refill_per_sec
        self._state = {}

    def allow(self, user: str) -> bool:
        now = time.monotonic()
        tokens, last = self._state.get(user, (self.cap, now))
        tokens = min(self.cap, tokens + (now - last) * self.refill)
        if tokens >= 1.0:
            self._state[user] = [tokens - 1.0, now]
            return True
        self._state[user] = [tokens, now]
        return False


limiter = RateLimiter(RL_CAPACITY, RL_REFILL)


# ----------------------------------------------------------------------------
# Auth middleware (pure ASGI so it covers WebSockets too)
# ----------------------------------------------------------------------------
class GatewayAuth:
    def __init__(self, app):
        self.app = app

    async def __call__(self, scope, receive, send):
        if scope["type"] not in ("http", "websocket"):
            return await self.app(scope, receive, send)

        headers = Headers(scope=scope)
        login = headers.get("tailscale-user-login") or DEV_USER
        path = scope.get("path", "")
        method = scope.get("method")

        if not login:
            return await self._deny(
                scope, receive, send, 403,
                "No Tailscale identity on this request. The gateway is reachable "
                "only over the tailnet via `tailscale serve`. "
                "(For local testing set GATEWAY_DEV_USER.)",
            )

        role = role_for(login)
        if role is None:
            audit(login, None, method or "WS", path, 403, "not on allowlist")
            return await self._deny(
                scope, receive, send, 403,
                f"{login} is not authorized. Add this login to {ALLOWLIST_FILE}.",
            )

        needed = required_role(method, path)
        if ROLE_RANK.get(role, 0) < ROLE_RANK[needed]:
            audit(login, role, method or "WS", path, 403, f"needs role '{needed}'")
            return await self._deny(
                scope, receive, send, 403,
                f"This action requires the '{needed}' role; you have '{role}'.",
            )

        if method in ("POST", "PUT", "DELETE", "PATCH") and not limiter.allow(login):
            audit(login, role, method, path, 429, "rate limited")
            return await self._deny(scope, receive, send, 429, "Rate limit exceeded.")

        # Hand the resolved identity down to the route handlers.
        scope = dict(scope)
        scope["gw_user"] = login
        scope["gw_role"] = role
        return await self.app(scope, receive, send)

    async def _deny(self, scope, receive, send, code, msg):
        if scope["type"] == "websocket":
            # 1008 = policy violation.
            await send({"type": "websocket.close", "code": 1008})
            return
        await PlainTextResponse(msg, status_code=code)(scope, receive, send)


# ----------------------------------------------------------------------------
# App
# ----------------------------------------------------------------------------
app = FastAPI(title="Go2 Gateway")
app.add_middleware(GatewayAuth)

_client: httpx.AsyncClient | None = None


@app.on_event("startup")
async def _startup():
    global _client
    _client = httpx.AsyncClient(timeout=httpx.Timeout(UPSTREAM_TIMEOUT, connect=5.0))
    log.info("Gateway up. Upstream Go2 = %s  (token=%s)",
             GO2_UPSTREAM, "set" if GO2_TOKEN else "NONE")
    log.info("Allowlist: %d user(s), default_role=%r", len(ALLOWLIST), DEFAULT_ROLE)
    if DEV_USER:
        log.warning("GATEWAY_DEV_USER=%s is set — identity check is bypassed. DEV ONLY.", DEV_USER)


@app.on_event("shutdown")
async def _shutdown():
    if _client:
        await _client.aclose()


def _upstream_headers(request: Request) -> dict:
    fwd = {
        k: v for k, v in request.headers.items()
        if k.lower() not in HOP_BY_HOP and not k.lower().startswith("tailscale-user")
    }
    if GO2_TOKEN:
        fwd["authorization"] = f"Bearer {GO2_TOKEN}"
    return fwd


# Pages — served by the gateway, straight from the robot repo's static tree.
app.mount("/app/static", StaticFiles(directory=str(ROBOT_STATIC)), name="static")


@app.get("/")
async def home():
    return FileResponse(str(GATEWAY_PAGES / "gateway_home.html"))


def _page(name):
    async def _serve():
        return FileResponse(str(PAGES_DIR / name))
    return _serve


app.add_api_route("/joystick", _page("go2_joystick.html"), methods=["GET"])
app.add_api_route("/movement", _page("go2_movement_controller.html"), methods=["GET"])
app.add_api_route("/rl_sim_to_real", _page("RL_sim_to_real.html"), methods=["GET"])
app.add_api_route("/camera", _page("go2_front_camera.html"), methods=["GET"])
app.add_api_route("/terminal", _page("go2_terminal_only.html"), methods=["GET"])
app.add_api_route("/other", _page("go2_other.html"), methods=["GET"])


@app.get("/config")
async def config():
    # The browser never holds a token here — identity comes from Tailscale — so
    # the shared UI's auth prompt is switched off.
    return JSONResponse({
        "auth_enabled": False,
        "deployment_env": "gateway",
        "default_page": "/",
    })


# Lets the page show which identity Tailscale resolved, and what it may do.
@app.get("/gateway/whoami")
async def whoami(request: Request):
    return JSONResponse({
        "user": request.scope.get("gw_user"),
        "role": request.scope.get("gw_role"),
    })


async def _probe_robot():
    url = f"{GO2_UPSTREAM}/health"
    headers = {"authorization": f"Bearer {GO2_TOKEN}"} if GO2_TOKEN else {}
    t0 = time.monotonic()
    try:
        r = await _client.get(url, headers=headers)
        ms = int((time.monotonic() - t0) * 1000)
        return {"robot_online": r.is_success, "status": r.status_code,
                "latency_ms": ms, "upstream": GO2_UPSTREAM, "detail": r.text[:200]}
    except Exception as exc:
        return {"robot_online": False, "status": 0, "latency_ms": None,
                "upstream": GO2_UPSTREAM, "detail": str(exc)}


@app.get("/gateway/status")
async def gateway_status(request: Request):
    res = await _probe_robot()
    audit(request.scope.get("gw_user"), request.scope.get("gw_role"),
          "GET", "/gateway/status", 200 if res["robot_online"] else 502)
    return JSONResponse(res)


@app.post("/gateway/connect")
async def gateway_connect(request: Request):
    """The 'Connect to Go2' button posts here: probe the robot over the tailnet."""
    res = await _probe_robot()
    audit(request.scope.get("gw_user"), request.scope.get("gw_role"),
          "POST", "/gateway/connect", 200 if res["robot_online"] else 502)
    return JSONResponse(res, status_code=200 if res["robot_online"] else 502)


# WebSocket proxy — pumps frames both ways between the client and the robot.
@app.websocket("/ws/{ws_path:path}")
async def ws_proxy(client_ws: WebSocket, ws_path: str):
    await client_ws.accept()

    q = dict(client_ws.query_params)
    if GO2_TOKEN:
        q["token"] = GO2_TOKEN
    qs = ("?" + urlencode(q)) if q else ""
    up_url = f"{WS_UPSTREAM}/ws/{ws_path}{qs}"

    user = client_ws.scope.get("gw_user")
    role = client_ws.scope.get("gw_role")
    audit(user, role, "WS", f"/ws/{ws_path}", 101)

    try:
        async with websockets.connect(up_url, max_size=None, ping_interval=None) as up:
            async def c2u():
                try:
                    while True:
                        msg = await client_ws.receive()
                        if msg.get("type") == "websocket.disconnect":
                            break
                        if msg.get("text") is not None:
                            await up.send(msg["text"])
                        elif msg.get("bytes") is not None:
                            await up.send(msg["bytes"])
                finally:
                    await up.close()

            async def u2c():
                try:
                    async for data in up:
                        if isinstance(data, str):
                            await client_ws.send_text(data)
                        else:
                            await client_ws.send_bytes(data)
                finally:
                    try:
                        await client_ws.close()
                    except RuntimeError:
                        pass

            await asyncio.gather(c2u(), u2c(), return_exceptions=True)
    except Exception as exc:
        audit(user, role, "WS", f"/ws/{ws_path}", 502, str(exc)[:200])
        try:
            await client_ws.close(code=1011)
        except RuntimeError:
            pass


# Catch-all HTTP proxy — must stay last so the named routes above win.
@app.api_route("/{full_path:path}",
               methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"])
async def http_proxy(full_path: str, request: Request):
    url = f"{GO2_UPSTREAM}/{full_path}"
    body = await request.body()
    user = request.scope.get("gw_user")
    role = request.scope.get("gw_role")
    try:
        upstream = await _client.request(
            request.method, url,
            params=dict(request.query_params),
            headers=_upstream_headers(request),
            content=body,
        )
    except Exception as exc:
        audit(user, role, request.method, "/" + full_path, 502, str(exc)[:200])
        return JSONResponse(
            {"error": "robot unreachable", "upstream": GO2_UPSTREAM, "detail": str(exc)},
            status_code=502,
        )

    resp_headers = {k: v for k, v in upstream.headers.items() if k.lower() not in HOP_BY_HOP}
    audit(user, role, request.method, "/" + full_path, upstream.status_code)
    return Response(content=upstream.content, status_code=upstream.status_code,
                    headers=resp_headers)
