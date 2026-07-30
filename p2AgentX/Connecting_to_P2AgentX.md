# Connecting to P2AgentX Backend

This guide outlines the EC2 workflow for the current **Go2RemoteConnection** repository and the new company-facing controller page.

## Important Information
- EC2 systemd service name: `go2-backend`
- Company URL: `https://p2dingo-control-backend.p2agentx.com`

### 📋 Your Instance Information

```
Instance ID:  i-0f7c599a52fac7e91
Region:       ap-southeast-2 (Sydney)
SSH Key:      aws/go2-robot-control-key.pem
Token:        See token.txt file
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

---

### 3. Create management script directory

```bash
mkdir -p ~/go2_ws/Go2RemoteConnection/p2AgentX
```

Place the three scripts below into that folder:

- `start_P2Dingo.sh`
- `stop_P2Dingo.sh`
- `status_P2Dingo.sh`

Make them executable:

```bash
chmod +x ~/go2_ws/Go2RemoteConnection/p2AgentX/start_P2Dingo.sh
chmod +x ~/go2_ws/Go2RemoteConnection/p2AgentX/stop_P2Dingo.sh
chmod +x ~/go2_ws/Go2RemoteConnection/p2AgentX/status_P2Dingo.sh
```

---

## 🎯 Daily Usage

### Start the EC2 backend

```bash
~/go2_ws/Go2RemoteConnection/p2AgentX/start_P2Dingo.sh
```
**Wait for it to show the IP address (about 60 seconds)**

This will:

1. start the EC2 instance
2. wait for it to boot
3. fetch the public IP
4. restart the `go2-backend` service on the EC2 host
5. print the company URL and useful checks

### Check status

```bash
~/go2_ws/Go2RemoteConnection/p2AgentX/status_P2Dingo.sh
```

### Stop the backend

```bash
~/go2_ws/Go2RemoteConnection/p2AgentX/stop_P2Dingo.sh
```

This will:

1. stop the `go2-backend` service on EC2
2. stop the EC2 instance

---

## Public URL

The company-facing controller page should be available at:

```text
https://p2dingo-control-backend.p2agentx.com/p2dingo
```

You can also keep `/` routed to that page if desired.

---

## Backend behavior for this deployment

The updated backend should:

- run on `0.0.0.0:8000`
- serve `P2Dingo_controller.html`
- serve static files from `/app/static`
- keep auth off by default
- expose `/config` so the frontend can discover whether auth is enabled
- optionally load a token from `~/go2_token` if auth is later enabled
- continue launching the full ROS/camera/bridge node stack

---

## Updating code on EC2

From your local machine:

```bash
# 1. Get current EC2 IP
cd ~/go2_ws/Go2RemoteConnection/p2AgentX
./check_P2Dingo.sh
# Note the IP (e.g., 54.66.38.113)
scp -i aws/go2-robot-control-key.pem -r src RL_start_remote_connection.sh ubuntu@YOUR_EC2_IP:/opt/go2-backend/
ssh -i aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP "sudo systemctl restart go2-backend"
```

Then verify:

```bash
ssh -i aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP "sudo systemctl status go2-backend"
ssh -i aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP "sudo journalctl -u go2-backend -n 100"
```

## Suggested service behavior

Your existing `go2-backend` service should launch the updated startup script.

---

## Health checks

If auth is off:

```bash
curl http://YOUR_EC2_IP:8000/health
```

If auth is enabled later:

```bash
curl http://YOUR_EC2_IP:8000/health -H "Authorization: Bearer $(cat ~/go2_token)"
```

Company URL check:

```bash
curl https://p2dingo-control-backend.p2agentx.com/health
```

---

## Useful troubleshooting

### Backend page loads but buttons do nothing

Check:

- EC2 instance is running
- `go2-backend` service is active
- reverse proxy is forwarding websocket traffic too
- backend is listening on port `8000`
- browser console shows no blocked mixed-content or websocket errors

### Camera or terminal does not work from company domain

This usually means the proxy is forwarding HTTP but not WebSockets.

Needed websocket endpoints include:

- `/ws/terminal`
- `/ws/cam_front`
- `/ws/cam_yolo`

### Wrong port used

The new company-facing page should use the current page origin by default, not hardcode `:8000`.

### Auth problems later

If you turn auth on:

- set `GO2_AUTH_ENABLED=1`
- load the token from `~/go2_token`
- enter the token in the page login overlay

---

## Quick command reference

```bash
# Start everything
~/go2_ws/Go2RemoteConnection/p2AgentX/start_P2Dingo.sh

# Check EC2 + service status
~/go2_ws/Go2RemoteConnection/p2AgentX/status_P2Dingo.sh

# Stop service + instance
~/go2_ws/Go2RemoteConnection/p2AgentX/stop_P2Dingo.sh
```

---

## Final notes

The new deployment model is:

- one FastAPI backend
- one company URL
- one controller page served directly by FastAPI
- no separate `http.server` frontend
- port `8000` on EC2
- auth off by default, optional later