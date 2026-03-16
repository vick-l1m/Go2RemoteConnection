/* shared.js
 * Shared helpers for Go2 web UIs:
 * - Auth/login overlay (optional, can be injected or use existing DOM)
 * - Login/logout buttons (toggleable)
 * - baseUrl/auth headers + localStorage
 * - httpGet/httpPost + health helper
 * - UI lock/unlock + status helper
 * - Terminal WebSocket helpers (xterm.js pages can use this)
 *
 * Usage:
 *   Go2Shared.init({
 *     defaultApi: "https://p2dingo-control-backend.p2agentx.com",
 *     showAuthButtons: true, // renders Login/Logout buttons into #authButtons if present
 *   });
 */

(function () {
  const Go2Shared = {};

  // ----------------------------
  // Config / State
  // ----------------------------
  Go2Shared.state = {
    COMMS_ENABLED: true,
    AUTH_TOKEN: "",
    ROBOT_BASE_URL: "",
  };

  Go2Shared.cfg = {
    defaultApi: "",
    showAuthButtons: false,
    // localStorage keys
    LS_URL_KEY: "go2_baseUrl",
    LS_TOKEN_KEY: "go2_token",
    authEnabled: true,
  };

  Go2Shared.fetchConfig = async function fetchConfig() {
    try {
      const r = await fetch(Go2Shared.baseUrl() + "/config", { cache: "no-store" });
      if (!r.ok) return null;
  
      const cfg = await r.json();
  
      if (cfg && typeof cfg.auth_enabled === "boolean") {
        Go2Shared.cfg.authEnabled = cfg.auth_enabled;
      }
  
      return cfg;
    } catch {
      return null;
    }
  };

  // ----------------------------
  // DOM helpers (safe)
  // ----------------------------
  function $(id) { return document.getElementById(id); }
  function setText(id, text) { const el = $(id); if (el) el.textContent = text; }
  function setHtml(id, html) { const el = $(id); if (el) el.innerHTML = html; }

  // ----------------------------
  // URL + auth
  // ----------------------------
  Go2Shared.baseUrl = function baseUrl() {
    const s = Go2Shared.state;
    const input = $("baseUrl");
    return (s.ROBOT_BASE_URL || (input ? input.value : "") || "")
      .trim()
      .replace(/\/+$/, "");
  };

  Go2Shared.authHeaders = function authHeaders() {
    if (!Go2Shared.cfg.authEnabled) return {};
    const t = Go2Shared.state.AUTH_TOKEN;
    return t ? { Authorization: `Bearer ${t}` } : {};
  };
  
  Go2Shared.apiWsBase = function apiWsBase() {
    const api = Go2Shared.baseUrl();               // e.g. http://192.168.x.x:8000 or https://...
    const u = new URL(api);
    const wsProto = u.protocol === "https:" ? "wss" : "ws";
    return `${wsProto}://${u.host}`;
  };

  Go2Shared.computeDefaultApi = function computeDefaultApi() {
    // If UI is opened as http://192.168.x.x:8081/... then hostname is 192.168.x.x
    const host = window.location.hostname; // no port
    const proto = window.location.protocol; // "http:" or "https:"
    const apiPort = 8000;
    return `${proto}//${host}:${apiPort}`;
  };


  // ----------------------------
  // UI lock/unlock
  // ----------------------------
  Go2Shared.lockUI = function lockUI() {
    const overlay = $("loginOverlay");
    if (overlay) overlay.style.display = "flex";

    const root = $("appRoot");
    if (root) {
      root.style.filter = "blur(6px)";
      root.style.pointerEvents = "none";
      root.style.userSelect = "none";
    }
  };

  Go2Shared.unlockUI = function unlockUI() {
    const overlay = $("loginOverlay");
    if (overlay) overlay.style.display = "none";

    const root = $("appRoot");
    if (root) {
      root.style.filter = "none";
      root.style.pointerEvents = "auto";
      root.style.userSelect = "auto";
    }
  };

  // ----------------------------
  // localStorage
  // ----------------------------
  Go2Shared.saveAuth = function saveAuth() {
    const { LS_URL_KEY, LS_TOKEN_KEY } = Go2Shared.cfg;
    localStorage.setItem(LS_URL_KEY, Go2Shared.state.ROBOT_BASE_URL);
    localStorage.setItem(LS_TOKEN_KEY, Go2Shared.state.AUTH_TOKEN);
  };

  Go2Shared.loadAuth = function loadAuth() {
    const { LS_URL_KEY, LS_TOKEN_KEY } = Go2Shared.cfg;
    return {
      u: localStorage.getItem(LS_URL_KEY) || "",
      t: localStorage.getItem(LS_TOKEN_KEY) || "",
    };
  };

  Go2Shared.clearAuth = function clearAuth() {
    const { LS_URL_KEY, LS_TOKEN_KEY } = Go2Shared.cfg;
    localStorage.removeItem(LS_URL_KEY);
    localStorage.removeItem(LS_TOKEN_KEY);
  };

  Go2Shared.ensureLoggedIn = function ensureLoggedIn() {
    const s = Go2Shared.state;
    
    if (!s.ROBOT_BASE_URL) {
      const msg = $("loginMsg");
      if (msg) msg.textContent = "Please enter robot URL.";
      Go2Shared.lockUI();
      return false;
    }

    if (Go2Shared.cfg.authEnabled && !s.AUTH_TOKEN) {
      const msg = $("loginMsg");
      if (msg) msg.textContent = "Please log in.";
      Go2Shared.lockUI();
      return false;
    }
    return true;
  };

  // ----------------------------
  // Status helper (shared “msg/detail/apiHost” pattern)
  // ----------------------------
  Go2Shared.setStatus = function setStatus(text, ok = true, detail = "") {
    const msg = $("msg");
    const detailEl = $("detail");
    if (msg) {
      msg.textContent = text;
      msg.style.color = ok ? "#0a7a0a" : "#b00020";
    }
    if (detailEl) detailEl.textContent = detail || "";

    const apiHost = $("apiHost");
    if (apiHost) apiHost.textContent = Go2Shared.baseUrl();
  };

  // ----------------------------
  // Comms gate (used by STOP latch logic)
  // ----------------------------
  Go2Shared.isAlwaysAllowed = function isAlwaysAllowed(path) {
    return path === "/health" || path.startsWith("/safety/");
  };

  Go2Shared.commsAllowed = function commsAllowed(path, opts = {}) {
    if (opts.bypassCommsGate) return true;
    if (Go2Shared.isAlwaysAllowed(path)) return true;
    return Go2Shared.state.COMMS_ENABLED;
  };

  // ----------------------------
  // HTTP helpers
  // ----------------------------
  Go2Shared.httpGet = async function httpGet(path, opts = {}) {
    const url = Go2Shared.baseUrl() + path;
  
    if (!Go2Shared.commsAllowed(path, opts)) {
      return { ok: false, status: 0, text: "Comms disabled", url };
    }
  
    const r = await fetch(url, { headers: Go2Shared.authHeaders() });
    const t = await r.text();
  
    // Only force relogin if auth is enabled
    if (Go2Shared.cfg.authEnabled && (r.status === 401 || r.status === 403)) {
      Go2Shared.state.AUTH_TOKEN = "";
      Go2Shared.clearAuth();
      Go2Shared.lockUI();
      const msg = $("loginMsg");
      if (msg) msg.textContent = "Invalid token. Please log in again.";
    }
  
    return { ok: r.ok, status: r.status, text: t, url };
  };

  Go2Shared.httpPost = async function httpPost(path, json = null, opts = {}) {
    const url = Go2Shared.baseUrl() + path;

    if (!Go2Shared.commsAllowed(path, opts)) {
      return { ok: false, status: 0, text: "Comms disabled", url };
    }

    const headers = { ...Go2Shared.authHeaders() };
    if (json !== null) headers["Content-Type"] = "application/json";

    const r = await fetch(url, {
      method: "POST",
      headers,
      body: json !== null ? JSON.stringify(json) : null,
    });
    const t = await r.text();

    if (r.status === 401 || r.status === 403) {
      Go2Shared.state.AUTH_TOKEN = "";
      Go2Shared.state.ROBOT_BASE_URL = "";
      Go2Shared.clearAuth();
      Go2Shared.lockUI();
      const msg = $("loginMsg");
      if (msg) msg.textContent = "Invalid token. Please log in again.";
    }

    return { ok: r.ok, status: r.status, text: t, url };
  };

  Go2Shared.checkHealth = async function checkHealth() {
    Go2Shared.setStatus("Checking /health...");
    const r = await Go2Shared.httpGet("/health");
    Go2Shared.setStatus(
      r.ok ? "Health OK ✅" : `Health failed (HTTP ${r.status})`,
      r.ok,
      `URL: ${r.url}\n${r.text}`
    );
    return r;
  };

  // ----------------------------
  // Login / Logout
  // ----------------------------
  Go2Shared.login = async function login() {
    const msg = $("loginMsg");
    if (msg) msg.textContent = "";

    const urlEl = $("loginBaseUrl");
    const tokenEl = $("loginToken");

    const url = urlEl ? urlEl.value.trim().replace(/\/+$/, "") : "";
    const token = tokenEl ? tokenEl.value.trim() : "";

    if (!url) {
      if (msg) msg.textContent = "Please enter robot URL.";
      Go2Shared.lockUI();
      return;
    }
    if (Go2Shared.cfg.authEnabled && !token) {
      if (msg) msg.textContent = "Please enter robot URL and password.";
      Go2Shared.lockUI();
      return;
    }

    // set state
    Go2Shared.state.ROBOT_BASE_URL = url;
    Go2Shared.state.AUTH_TOKEN = token;

    // reflect to main input
    const baseInput = $("baseUrl");
    if (baseInput) baseInput.value = url;
    setText("apiHost", url);

    const r = await Go2Shared.httpGet("/health").catch((e) => ({
      ok: false, status: 0, text: String(e), url: Go2Shared.baseUrl() + "/health"
    }));

    if (!r.ok) {
      if (msg) {
        msg.textContent =
          (r.status === 401 || r.status === 403) ? "Incorrect token entered."
          : `Login failed (HTTP ${r.status})`;
      }
      Go2Shared.state.AUTH_TOKEN = "";
      Go2Shared.state.ROBOT_BASE_URL = "";
      Go2Shared.clearAuth();
      Go2Shared.lockUI();
      return;
    }

    Go2Shared.saveAuth();
    Go2Shared.unlockUI();
    Go2Shared.setStatus("Unlocked ✅", true);
  };

  Go2Shared.logout = function logout() {
    Go2Shared.state.AUTH_TOKEN = "";
    Go2Shared.state.ROBOT_BASE_URL = "";
    Go2Shared.clearAuth();

    const tokenEl = $("loginToken");
    if (tokenEl) tokenEl.value = "";

    const cb = $("showPw");
    if (cb) cb.checked = false;
    if (tokenEl) tokenEl.type = "password";

    Go2Shared.lockUI();
    Go2Shared.setStatus("Logged out.", true);
  };

  Go2Shared.togglePasswordVisibility = function togglePasswordVisibility() {
    const pw = $("loginToken");
    const cb = $("showPw");
    if (!pw || !cb) return;
    pw.type = cb.checked ? "text" : "password";
  };

  Go2Shared.wireEnterToUnlock = function wireEnterToUnlock() {
    const urlEl = $("loginBaseUrl");
    const passEl = $("loginToken");
    const btn = $("unlockBtn");

    const handler = (e) => {
      if (e.key === "Enter") {
        e.preventDefault();
        if (btn) btn.click();
        else Go2Shared.login();
      }
    };

    if (urlEl) urlEl.addEventListener("keydown", handler);
    if (passEl) passEl.addEventListener("keydown", handler);
  };

  // ----------------------------
  // Optional: render Login/Logout buttons into #authButtons
  // (toggleable via cfg.showAuthButtons)
  // ----------------------------
  Go2Shared.renderAuthButtons = function renderAuthButtons() {
    if (!Go2Shared.cfg.showAuthButtons) return;

    const host = $("authButtons");
    if (!host) return;

    host.innerHTML = `
      <button id="loginBtnInline">Login</button>
      <button id="logoutBtnInline">Logout</button>
    `;

    $("loginBtnInline")?.addEventListener("click", () => Go2Shared.lockUI());
    $("logoutBtnInline")?.addEventListener("click", () => Go2Shared.logout());
  };

  // ----------------------------
  // Auto-init (load remembered auth, set default URL, lock/unlock)
  // ----------------------------
  Go2Shared.init = async function init({ defaultApi = "", showAuthButtons = false } = {}) {
    if (!defaultApi) defaultApi = Go2Shared.computeDefaultApi();
  
    Go2Shared.cfg.defaultApi = defaultApi;
    Go2Shared.cfg.showAuthButtons = !!showAuthButtons;
  
    setText("uiHost", location.origin);
    
    Go2Shared.injectNavBar();
    Go2Shared.highlightActivePage();
    
    // Fill inputs (no hardcoded value in HTML needed)
    const loginUrl = $("loginBaseUrl");
    const baseInput = $("baseUrl");
    if (loginUrl) loginUrl.value = defaultApi;
    if (baseInput) baseInput.value = defaultApi;
    setText("apiHost", defaultApi);
  
    Go2Shared.renderAuthButtons();
    Go2Shared.wireEnterToUnlock();
  
    // Make sure baseUrl() works for /config fetch
    Go2Shared.state.ROBOT_BASE_URL = defaultApi;
  
    const cfg = await Go2Shared.fetchConfig();
    if (cfg && typeof cfg.auth_enabled === "boolean") {
      Go2Shared.cfg.authEnabled = cfg.auth_enabled;
    }
  
    // ✅ Set dataset for CSS rules
    document.documentElement.dataset.authEnabled = Go2Shared.cfg.authEnabled ? "1" : "0";

    // ✅ If auth disabled, never show overlay
    if (!Go2Shared.cfg.authEnabled) {
      Go2Shared.state.AUTH_TOKEN = "";
      Go2Shared.unlockUI();                 // hide overlay + unlock
      Go2Shared.injectNavBar();             // still show nav
      Go2Shared.highlightActivePage();
      return;
    }
  
    // Auth enabled: try remembered login
    const { u, t } = Go2Shared.loadAuth();
    if (u && t) {
      Go2Shared.state.ROBOT_BASE_URL = u;
      Go2Shared.state.AUTH_TOKEN = t;
      if (loginUrl) loginUrl.value = u;
      const loginToken = $("loginToken");
      if (loginToken) loginToken.value = t;
      if (baseInput) baseInput.value = u;
      setText("apiHost", u);
  
      const r = await Go2Shared.httpGet("/health").catch(() => ({ ok: false }));
      if (r.ok) {
        Go2Shared.unlockUI();
        Go2Shared.setStatus("Unlocked ✅ (remembered)", true);
        return;
      }
    }

    // No remembered login or it failed => lock
    Go2Shared.lockUI();
  };

  // ----------------------------
  // Terminal WS helpers (xterm page can build on this)
  // ----------------------------
  Go2Shared.connectTerminalWs = function connectTerminalWs(st, n, onOpenCb) {
    if (!st || !st.term) return;

    // If auth enabled, require token; if disabled, connect without it
    let wsUrl = `${Go2Shared.apiWsBase()}/ws/terminal`;

    if (Go2Shared.cfg.authEnabled) {
      if (!Go2Shared.state.AUTH_TOKEN) {
        st.term.write("\r\n[Login required]\r\n");
        return;
      }
      wsUrl += `?token=${encodeURIComponent(Go2Shared.state.AUTH_TOKEN)}`;
    }

    // already connected/connecting?
    if (st.ws && (st.ws.readyState === WebSocket.OPEN || st.ws.readyState === WebSocket.CONNECTING)) return;

    st.ws = new WebSocket(wsUrl);

    st.ws.onopen = () => {
      st.term.write(`\r\n[Connected to Go2 terminal ${n}]\r\n`, () => {
        onOpenCb && onOpenCb();
        st.term.focus();
      });
    };

    st.ws.onmessage = (e) => {
      st.term.write(e.data);
    };

    st.ws.onclose = () => {
      st.term && st.term.write("\r\n[Terminal disconnected]\r\n");
      st.ws = null;
    };

    st.ws.onerror = () => {
      st.term && st.term.write("\r\n[Terminal error]\r\n");
    };
  };

  // ----------------------------
  // Navigation bar + active page highlight
  // ----------------------------
  Go2Shared.highlightActivePage = function highlightActivePage() {
    const path = location.pathname.split("/").pop() || "";
    document.querySelectorAll(".pageNav .navBtn").forEach(a => {
      const href = (a.getAttribute("href") || "").split("/").pop();
      a.classList.toggle("active", href === path);
    });
  };

  Go2Shared.injectNavBar = function injectNavBar() {
    const card = document.querySelector("#appRoot .card");
    if (!card) return;
  
    if (card.querySelector(".pageNav")) return;
  
    const nav = document.createElement("div");
    nav.className = "pageNav";
    nav.innerHTML = `
      <a class="navBtn" href="/app/go2_joystick.html">Joysticks</a>
      <a class="navBtn" href="/app/go2_movement_controller.html">Movement</a>
      <a class="navBtn" href="/app/go2_map_viewer.html">Map</a>
      <a class="navBtn" href="/app/go2_terminal_only.html">Terminal</a>
      <a class="navBtn" href="/app/go2_other.html">Other</a>
      <a class="navBtn" href="/app/go2_front_camera.html">Camera</a>
`   ;
  
    const h1 = card.querySelector("h1");
    if (h1 && h1.nextSibling) card.insertBefore(nav, h1.nextSibling);
    else card.prepend(nav);
  
    Go2Shared.highlightActivePage();
  };

  // Expose
  window.Go2Shared = Go2Shared;
})();



