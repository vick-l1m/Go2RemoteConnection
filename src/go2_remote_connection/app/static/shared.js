/*
shared.js

This file provides shared frontend utilities for all Go2 web pages.
It includes:
- login/logout handling
- API base URL and token management
- shared status display
- health checks
- GET/POST helpers
- shared route-based nav bar injection
- active nav highlighting
- terminal WebSocket helper
- shared joystick helper

Version 2.1
Author: Victor Lim
*/

window.Go2Shared = {
  state: {
    API_BASE: "",
    AUTH_TOKEN: "",
    AUTH_ENABLED: true,
    COMMS_ENABLED: true,
    SHOW_AUTH_BUTTONS: true,
  },

  async init(opts = {}) {
    const defaultApi = opts.defaultApi ?? window.location.origin;
    const showAuthButtons = opts.showAuthButtons ?? true;

    this.state.SHOW_AUTH_BUTTONS = !!showAuthButtons;

    const savedBase = localStorage.getItem("go2_api_base") || defaultApi || "";
    this.state.API_BASE = this.normalizeBase(savedBase);
    this.state.AUTH_TOKEN = localStorage.getItem("go2_auth_token") || "";

    if (this.state.API_BASE) {
      localStorage.setItem("go2_api_base", this.state.API_BASE);
    }

    const baseInput = document.getElementById("baseUrl");
    if (baseInput) baseInput.value = this.state.API_BASE;

    const loginBase = document.getElementById("loginBaseUrl");
    if (loginBase) loginBase.value = this.state.API_BASE;

    const loginToken = document.getElementById("loginToken");
    if (loginToken) loginToken.value = this.state.AUTH_TOKEN;

    this.injectNavBar();
    this.highlightActivePage();

    await this.fetchConfigMaybe();
    this.updateAuthUi();
  },

  normalizeBase(base) {
    base = (base || "").trim().replace(/\/+$/, "");

    if (!base) {
      return window.location.origin;
    }

    if (!/^https?:\/\//i.test(base)) {
      base = "http://" + base;
    }

    try {
      const u = new URL(base);

      // Default to :8000 if user only typed host/IP
      if (!u.port) {
        u.port = "8000";
      }

      return u.origin;
    } catch (_) {
      return base;
    }
  },

  apiBase() {
    const baseInput = document.getElementById("baseUrl");
    const raw = baseInput ? baseInput.value : this.state.API_BASE;
    const base = this.normalizeBase(raw);

    this.state.API_BASE = base;
    if (baseInput) baseInput.value = base;
    if (base) localStorage.setItem("go2_api_base", base);

    const loginBase = document.getElementById("loginBaseUrl");
    if (loginBase && !loginBase.value.trim()) {
      loginBase.value = base;
    }

    return base;
  },

  apiWsBase() {
    const base = this.apiBase();
    if (!base) return "";

    if (base.startsWith("https://")) {
      return "wss://" + base.slice("https://".length);
    }
    if (base.startsWith("http://")) {
      return "ws://" + base.slice("http://".length);
    }
    return base;
  },

  authHeaders() {
    const headers = { "Content-Type": "application/json" };
    if (this.state.AUTH_ENABLED && this.state.AUTH_TOKEN) {
      headers["Authorization"] = `Bearer ${this.state.AUTH_TOKEN}`;
    }
    return headers;
  },

  setStatus(message, ok = true, detail = "") {
    const msgEl = document.getElementById("msg");
    const detailEl = document.getElementById("detail");

    if (msgEl) msgEl.textContent = message || "";
    if (detailEl) detailEl.textContent = detail || "";

    if (msgEl) {
      msgEl.style.color = ok ? "#1f5f2c" : "#b00020";
    }
  },

  injectNavBar() {
    const root = document.getElementById("pageNavMount") || document.querySelector(".card");
    if (!root) return;
    if (document.querySelector(".pageNav")) return;

    const nav = document.createElement("div");
    nav.className = "pageNav";
    nav.innerHTML = `
      <a class="navBtn" href="/">Home</a>
      <a class="navBtn" href="/joystick">Joysticks</a>
      <a class="navBtn" href="/movement">Movement</a>
      <a class="navBtn" href="/map">Map</a>
      <a class="navBtn" href="/terminal">Terminal</a>
      <a class="navBtn" href="/other">Other</a>
      <a class="navBtn" href="/camera">Camera</a>
    `;

    if (root.id === "pageNavMount") {
      root.appendChild(nav);
    } else {
      const h1 = root.querySelector("h1");
      if (h1 && h1.nextSibling) root.insertBefore(nav, h1.nextSibling);
      else root.appendChild(nav);
    }
  },

  highlightActivePage() {
    const current = location.pathname.replace(/\/+$/, "") || "/";
    document.querySelectorAll(".pageNav .navBtn").forEach((a) => {
      const href = (a.getAttribute("href") || "").replace(/\/+$/, "") || "/";
      a.classList.toggle("active", href === current);
    });
  },

  updateAuthUi() {
    const overlay = document.getElementById("loginOverlay");
    const appRoot = document.getElementById("appRoot");

    if (!this.state.AUTH_ENABLED) {
      if (overlay) overlay.style.display = "none";
      if (appRoot) appRoot.classList.remove("appLocked");
      return;
    }

    const loggedIn = !!this.state.AUTH_TOKEN;

    if (overlay) overlay.style.display = loggedIn ? "none" : "flex";
    if (appRoot) {
      if (loggedIn) appRoot.classList.remove("appLocked");
      else appRoot.classList.add("appLocked");
    }
  },

  ensureLoggedIn() {
    if (!this.state.AUTH_ENABLED) return true;
    if (this.state.AUTH_TOKEN) return true;

    this.setStatus("Login required", false);
    const overlay = document.getElementById("loginOverlay");
    if (overlay) overlay.style.display = "flex";
    return false;
  },

  async fetchConfigMaybe() {
    const base = this.apiBase();
    if (!base) return;

    try {
      const r = await fetch(`${base}/config`, {
        method: "GET",
        headers: this.authHeaders(),
      });
      if (!r.ok) return;

      const data = await r.json();
      if (typeof data.auth_enabled === "boolean") {
        this.state.AUTH_ENABLED = data.auth_enabled;
      }
    } catch (_) {}
  },

  async login() {
    const baseInput = document.getElementById("loginBaseUrl");
    const tokenInput = document.getElementById("loginToken");
    const msg = document.getElementById("loginMsg");

    const base = this.normalizeBase(baseInput ? baseInput.value : "");
    const token = (tokenInput ? tokenInput.value : "").trim();

    this.state.API_BASE = base;
    this.state.AUTH_TOKEN = token;

    if (base) localStorage.setItem("go2_api_base", base);
    else localStorage.removeItem("go2_api_base");

    if (token) localStorage.setItem("go2_auth_token", token);
    else localStorage.removeItem("go2_auth_token");

    const mainBase = document.getElementById("baseUrl");
    if (mainBase) mainBase.value = base;

    if (baseInput) baseInput.value = base;

    await this.fetchConfigMaybe();

    if (!base) {
      if (msg) msg.textContent = "Enter a base URL.";
      this.updateAuthUi();
      return false;
    }

    if (this.state.AUTH_ENABLED && !token) {
      if (msg) msg.textContent = "Enter the robot password.";
      this.updateAuthUi();
      return false;
    }

    const r = await this.httpGet("/health", {
      bypassCommsGate: true,
      suppressStatus: true,
    });

    if (!r.ok) {
      if (msg) msg.textContent = `Connection failed (${r.status || "network"})`;
      this.setStatus("Login failed", false, `URL: ${r.url}\n${r.text}`);
      this.updateAuthUi();
      return false;
    }

    if (msg) msg.textContent = "";
    this.setStatus("Connected ✅", true);
    this.updateAuthUi();
    return true;
  },

  logout() {
    this.state.AUTH_TOKEN = "";
    localStorage.removeItem("go2_auth_token");

    const tokenInput = document.getElementById("loginToken");
    if (tokenInput) tokenInput.value = "";

    const msg = document.getElementById("loginMsg");
    if (msg) msg.textContent = "";

    this.updateAuthUi();
    this.setStatus("Logged out", true);
  },

  togglePasswordVisibility() {
    const input = document.getElementById("loginToken");
    if (!input) return;
    input.type = input.type === "password" ? "text" : "password";
  },

  async checkHealth() {
    const r = await this.httpGet("/health", { bypassCommsGate: true });
    this.setStatus(
      r.ok ? "Health OK ✅" : `Health failed (${r.status || "network"})`,
      r.ok,
      `URL: ${r.url}\n${r.text}`
    );
    return r;
  },

  async httpGet(path, opts = {}) {
    const { bypassCommsGate = false, suppressStatus = false } = opts;

    if (!bypassCommsGate && !this.state.COMMS_ENABLED) {
      return { ok: false, status: 0, text: "Comms disabled by safety latch", url: path };
    }

    const base = this.apiBase();
    const url = `${base}${path}`;

    try {
      const r = await fetch(url, {
        method: "GET",
        headers: this.authHeaders(),
      });

      const text = await r.text();

      if (!suppressStatus && !r.ok) {
        this.setStatus(`GET failed (${r.status})`, false, `URL: ${url}\n${text}`);
      }

      return { ok: r.ok, status: r.status, text, url };
    } catch (err) {
      if (!suppressStatus) {
        this.setStatus("GET network error", false, `URL: ${url}\n${String(err)}`);
      }
      return { ok: false, status: 0, text: String(err), url };
    }
  },

  async httpPost(path, body = null, opts = {}) {
    const { bypassCommsGate = false, suppressStatus = false } = opts;

    if (!bypassCommsGate && !this.state.COMMS_ENABLED) {
      return { ok: false, status: 0, text: "Comms disabled by safety latch", url: path };
    }

    const base = this.apiBase();
    const url = `${base}${path}`;

    try {
      const r = await fetch(url, {
        method: "POST",
        headers: this.authHeaders(),
        body: body == null ? null : JSON.stringify(body),
      });

      const text = await r.text();

      if (!suppressStatus && !r.ok) {
        this.setStatus(`POST failed (${r.status})`, false, `URL: ${url}\n${text}`);
      }

      return { ok: r.ok, status: r.status, text, url };
    } catch (err) {
      if (!suppressStatus) {
        this.setStatus("POST network error", false, `URL: ${url}\n${String(err)}`);
      }
      return { ok: false, status: 0, text: String(err), url };
    }
  },

  connectTerminalWs({ containerId, statusId, TerminalCtor, FitAddonCtor }) {
    const container = document.getElementById(containerId);
    const statusEl = document.getElementById(statusId);

    if (!container) {
      this.setStatus(`Missing terminal container: ${containerId}`, false);
      return null;
    }

    const term = new TerminalCtor({
      cursorBlink: true,
      convertEol: true,
      scrollback: 2000,
      fontSize: 13,
      fontFamily: "ui-monospace, SFMono-Regular, Menlo, Consolas, monospace",
      theme: {
        background: "#0f0f0f",
      },
    });

    const fitAddon = new FitAddonCtor();
    term.loadAddon(fitAddon);
    term.open(container);
    fitAddon.fit();

    const token = encodeURIComponent(this.state.AUTH_TOKEN || "");
    const tokenSuffix = (this.state.AUTH_ENABLED && token) ? `?token=${token}` : "";
    const ws = new WebSocket(`${this.apiWsBase()}/ws/terminal${tokenSuffix}`);

    ws.onopen = () => {
      if (statusEl) statusEl.textContent = "Connected";
      term.writeln("\r\n[connected]\r\n");
      fitAddon.fit();
      try {
        ws.send(JSON.stringify({
          cols: term.cols,
          rows: term.rows,
          kind: "resize",
        }));
      } catch (_) {}
    };

    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") {
        term.write(ev.data);
      }
    };

    ws.onclose = (ev) => {
      if (statusEl) statusEl.textContent = `Closed (${ev.code})`;
      term.writeln(`\r\n[closed ${ev.code}]\r\n`);
    };

    ws.onerror = () => {
      if (statusEl) statusEl.textContent = "Error";
      term.writeln("\r\n[websocket error]\r\n");
    };

    term.onData((data) => {
      if (ws.readyState === WebSocket.OPEN) {
        ws.send(data);
      }
    });

    function sendResize() {
      try {
        fitAddon.fit();
        if (ws.readyState === WebSocket.OPEN) {
          ws.send(JSON.stringify({
            cols: term.cols,
            rows: term.rows,
            kind: "resize",
          }));
        }
      } catch (_) {}
    }

    const resizeHandler = () => sendResize();
    window.addEventListener("resize", resizeHandler);
    setTimeout(sendResize, 100);

    return {
      term,
      fitAddon,
      ws,
      close() {
        window.removeEventListener("resize", resizeHandler);
        try { ws.close(); } catch (_) {}
        try { term.dispose(); } catch (_) {}
      },
    };
  },

  createJoystick(canvasId, onMove) {
    const canvas = document.getElementById(canvasId);
    if (!canvas) {
      this.setStatus(`Missing joystick canvas: ${canvasId}`, false);
      return null;
    }

    const ctx = canvas.getContext("2d");

    let active = false;
    let cx = 0;
    let cy = 0;
    let knobX = 0;
    let knobY = 0;
    let radius = 0;

    function resize() {
      const rect = canvas.getBoundingClientRect();
      if (!rect.width || !rect.height) return;

      canvas.width = rect.width * window.devicePixelRatio;
      canvas.height = rect.height * window.devicePixelRatio;
      ctx.setTransform(window.devicePixelRatio, 0, 0, window.devicePixelRatio, 0, 0);

      cx = rect.width / 2;
      cy = rect.height / 2;
      radius = Math.min(rect.width, rect.height) * 0.32;

      if (!active) {
        knobX = cx;
        knobY = cy;
      }

      draw();
    }

    function draw() {
      const rect = canvas.getBoundingClientRect();
      ctx.clearRect(0, 0, rect.width, rect.height);

      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);
      ctx.strokeStyle = "#bbb";
      ctx.lineWidth = 2;
      ctx.stroke();

      ctx.beginPath();
      ctx.arc(knobX, knobY, radius * 0.32, 0, Math.PI * 2);
      ctx.fillStyle = "#d9e8ff";
      ctx.strokeStyle = "#7aa7ff";
      ctx.lineWidth = 2;
      ctx.fill();
      ctx.stroke();
    }

    function updateFromEvent(clientX, clientY) {
      const rect = canvas.getBoundingClientRect();
      const x = clientX - rect.left;
      const y = clientY - rect.top;

      const dx = x - cx;
      const dy = y - cy;
      const dist = Math.hypot(dx, dy);

      let nx = dx;
      let ny = dy;

      if (dist > radius) {
        nx = (dx / dist) * radius;
        ny = (dy / dist) * radius;
      }

      knobX = cx + nx;
      knobY = cy + ny;
      draw();

      onMove(nx / radius, ny / radius, true);
    }

    function reset() {
      active = false;
      knobX = cx;
      knobY = cy;
      draw();
      onMove(0, 0, false);
    }

    canvas.addEventListener("pointerdown", (e) => {
      active = true;
      canvas.setPointerCapture(e.pointerId);
      updateFromEvent(e.clientX, e.clientY);
    });

    canvas.addEventListener("pointermove", (e) => {
      if (!active) return;
      updateFromEvent(e.clientX, e.clientY);
    });

    canvas.addEventListener("pointerup", reset);
    canvas.addEventListener("pointercancel", reset);

    window.addEventListener("resize", resize);
    resize();

    return {
      canvas,
      resize,
      reset,
      destroy() {
        window.removeEventListener("resize", resize);
      },
    };
  },
};