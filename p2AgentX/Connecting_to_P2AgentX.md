# Connecting to P2AgentX Backend

This guide covers the P2Dingo deployment: a standalone, company-facing controller
page served by its own FastAPI app, plus the AWS scripts that power the EC2 host
it runs on.

> **This folder holds two unrelated layers.** Keep them straight:
>
> | | Runs where | What it touches |
> |---|---|---|
> | `start/stop/status_P2Dingo.sh` | your laptop | **only** EC2 instance power (`aws ec2 …`) |
> | `start_p2dingo_backend.sh` | the robot or the EC2 host | the ROS 2 + FastAPI stack |
>
> The three AWS scripts do **not** SSH anywhere and do **not** start, stop, or
> restart the backend. They turn the box on and off. Bringing the software up is
> a separate step (systemd on EC2, or running the launcher by hand).

## Important Information

- EC2 systemd service name: `go2-backend`
- Company URL: `https://p2dingo-control-backend.p2agentx.com`
- Backend listens on **port 8200** (set by `API_PORT` in `start_p2dingo_backend.sh`)

### 📋 Instance Information

```
Instance ID:  i-0f7c599a52fac7e91
Region:       ap-southeast-2 (Sydney)
SSH Key:      NOT in this repo — see "SSH access" below
```

---

## 🚀 One-Time Setup (First Time Only)

### **Step 1: Install AWS CLI**

**Mac:**
```bash
brew install awscli
```

**Windows/Linux:**
```bash
pip3 install awscli
```

**Verify:**
```bash
aws --version
```

---

### **Step 2: Configure AWS Credentials**

```bash
aws configure
```

**Enter when asked:**
- AWS Access Key ID: `(ask your AWS admin)`
- AWS Secret Access Key: `(ask your AWS admin)`
- Default region: `ap-southeast-2`
- Default output format: `json`

Confirm the credentials resolve:

```bash
aws sts get-caller-identity
```

---

### **Step 3: Make the scripts executable**

The scripts ship with this repo in `p2AgentX/` — you do not need to create the
directory or copy anything in.

```bash
chmod +x ~/go2_ws/Go2RemoteConnection/p2AgentX/*.sh
```

---

### **Step 4: SSH access (only needed to deploy code)**

The private key is **not** committed — `p2AgentX/aws/go2-robot-control-key.pem`
was removed in `5724dfa`, and the file that was there only ever contained a
placeholder path, not key material. Get the real `.pem` from your AWS admin,
store it outside the repo, and lock it down:

```bash
chmod 600 /path/to/go2-robot-control-key.pem
```

Everything below that uses `$KEY` assumes you have set:

```bash
export KEY=/path/to/go2-robot-control-key.pem
```

---

## 🎯 Daily Usage

### Start the EC2 instance

```bash
~/go2_ws/Go2RemoteConnection/p2AgentX/start_P2Dingo.sh
```

**Takes roughly 60 seconds** — it blocks on two AWS waiters.

This will:

1. `aws ec2 start-instances`
2. wait for state `running`
3. wait for status checks to pass (`instance-status-ok`)
4. fetch and print the public IP
5. print the company URL and the health-check commands

It does **not** touch the `go2-backend` service. If that service is enabled on
the instance, systemd starts it on boot; otherwise bring it up yourself:

```bash
ssh -i "$KEY" ubuntu@YOUR_EC2_IP "sudo systemctl start go2-backend"
```

### Check status

```bash
~/go2_ws/Go2RemoteConnection/p2AgentX/status_P2Dingo.sh
```

Prints instance state, and the public IP when it is running.

### Stop the EC2 instance

```bash
~/go2_ws/Go2RemoteConnection/p2AgentX/stop_P2Dingo.sh
```

This runs `aws ec2 stop-instances` and waits for `stopped`. It does **not** stop
the service first — stopping the instance takes the service down with it. To
shut the service down cleanly beforehand:

```bash
ssh -i "$KEY" ubuntu@YOUR_EC2_IP "sudo systemctl stop go2-backend"
```

---

## Public URL

`p2dingo_main.py` serves the controller page at the **root** path:

```text
https://p2dingo-control-backend.p2agentx.com/
```

There is no `/p2dingo` route in the FastAPI app — that path works only if the
reverse proxy in front of it rewrites to `/`. (`start_P2Dingo.sh` still prints a
`/p2dingo` link, and `app/main.py`'s `/config` still advertises
`"default_page": "/p2dingo"`; both predate the app serving the page at `/`.)

---

## Backend behaviour for this deployment

`start_p2dingo_backend.sh` launches `app.p2dingo_main:app`, which:

- runs on `0.0.0.0:8200`
- serves `P2Dingo_controller.html` at `/`
- serves static files from `/app/static`
- keeps auth off by default (`GO2_AUTH_ENABLED=0`)
- exposes `/config` so the frontend can discover whether auth is enabled
- optionally loads a token from `~/go2_token` when `GO2_AUTH_ENABLED=1`
- brings up the ROS stack: `flatten_l1_data` (L1 lidar → `/map2d`), the front
  camera capture + ROS bridge, `web_bridge`, and `move_forward_meters_node`

Note this launcher starts the **lidar and camera** nodes and does **not** start
the RL policy nodes — that is the opposite of `RL_start_remote_connection.sh`.

`p2dingo_main.py` still includes `map_routes`, which `app/main.py` no longer
does. P2Dingo is now the only consumer of the map code; do not delete
`app/api/map_routes.py` as dead.

Nothing invokes `start_p2dingo_backend.sh` automatically. Point the
`go2-backend` systemd unit at it, or run it by hand.

---

## Updating code on EC2

```bash
# 1. Get the current EC2 IP
~/go2_ws/Go2RemoteConnection/p2AgentX/status_P2Dingo.sh
# Note the IP (e.g. 54.66.38.113)

# 2. Push the source and restart
cd ~/go2_ws/Go2RemoteConnection
scp -i "$KEY" -r src p2AgentX/start_p2dingo_backend.sh ubuntu@YOUR_EC2_IP:/opt/go2-backend/
ssh -i "$KEY" ubuntu@YOUR_EC2_IP "sudo systemctl restart go2-backend"
```

The FastAPI app is run straight from source (`python3 -m uvicorn app.p2dingo_main:app`),
so Python changes under `src/go2_remote_connection/app/` take effect on restart
with no rebuild. Changes to the **ROS nodes** — especially the C++ `web_bridge` —
need a `colcon build` on the target before they take effect.

Then verify:

```bash
ssh -i "$KEY" ubuntu@YOUR_EC2_IP "sudo systemctl status go2-backend"
ssh -i "$KEY" ubuntu@YOUR_EC2_IP "sudo journalctl -u go2-backend -n 100"
```

---

## Health checks

If auth is off (the default):

```bash
curl http://YOUR_EC2_IP:8200/health
```

If auth is enabled:

```bash
curl http://YOUR_EC2_IP:8200/health -H "Authorization: Bearer $(cat ~/go2_token)"
```

Company URL check (through the reverse proxy, so no port):

```bash
curl https://p2dingo-control-backend.p2agentx.com/health
```

---

## Useful troubleshooting

### Backend page loads but buttons do nothing

Check:

- EC2 instance is running (`status_P2Dingo.sh`)
- `go2-backend` service is active
- reverse proxy is forwarding websocket traffic too
- backend is listening on port `8200`
- browser console shows no blocked mixed-content or websocket errors

### Camera or terminal does not work from the company domain

This usually means the proxy is forwarding HTTP but not WebSockets.

Needed websocket endpoints include:

- `/ws/terminal`
- `/ws/cam_front`
- `/ws/cam_yolo`

### The launcher exits immediately

`start_p2dingo_backend.sh` treats **every** child as critical: if
`flatten_l1_data`, either camera node, or the API dies, `ok_or_die` tears the
whole stack down. Check the per-process logs it prints on exit
(`/tmp/*_p2dingo.log`). Note that `RL_start_remote_connection.sh` on `main` has
since gained a non-critical process tier; this launcher has not.

### Auth problems

If you turn auth on:

- set `GO2_AUTH_ENABLED=1`
- create `~/go2_token` on the host (the launcher aborts if it is missing)
- enter the token in the page login overlay

---

## Quick command reference

```bash
# Power the EC2 box on / off / query  (AWS only — no service control)
~/go2_ws/Go2RemoteConnection/p2AgentX/start_P2Dingo.sh
~/go2_ws/Go2RemoteConnection/p2AgentX/stop_P2Dingo.sh
~/go2_ws/Go2RemoteConnection/p2AgentX/status_P2Dingo.sh

# Bring the stack up on the robot / EC2 host
~/go2_ws/Go2RemoteConnection/p2AgentX/start_p2dingo_backend.sh
```

---

## Final notes

The deployment model is:

- one FastAPI backend (`app.p2dingo_main`)
- one company URL
- one controller page served at `/`
- no separate `http.server` frontend
- port `8200` on the host, fronted by the reverse proxy on 443
- auth off by default, optional later
