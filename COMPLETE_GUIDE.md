# Complete Robot Backend Guide

**Everything you need in one document**

---

## 📋 Your Instance Information

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

### **Step 3: Create Helper Scripts**

**Create these 3 scripts for easy management:**

#### **A) Create `start-robot.sh`**

```bash
nano ~/start-robot.sh
```

**Copy and paste this:**

```bash
#!/bin/bash
INSTANCE_ID="i-0f7c599a52fac7e91"

echo "🚀 Starting EC2 instance..."
aws ec2 start-instances --instance-ids $INSTANCE_ID

echo "⏳ Waiting 60 seconds for instance to start..."
sleep 60

PUBLIC_IP=$(aws ec2 describe-instances --instance-ids $INSTANCE_ID \
  --query 'Reservations[0].Instances[0].PublicIpAddress' \
  --output text)

echo ""
echo "✅ Instance started!"
echo "📍 Public IP: $PUBLIC_IP"
echo "🌐 Backend URL: http://$PUBLIC_IP"
echo ""
echo "📋 Next steps:"
echo "1. Test backend:"
echo "   curl http://$PUBLIC_IP/health -H \"Authorization: Bearer \$(cat ~/Documents/P2RemoteConnection/token.txt)\""
echo ""
echo "2. Start frontend:"
echo "   cd ~/Documents/P2RemoteConnection/src/p2_remote_connection/app"
echo "   python3 -m http.server 8080"
echo "   Open: http://localhost:8080/go2_joystick.html"
```

**Make it executable:**
```bash
chmod +x ~/start-robot.sh
```

---

#### **B) Create `stop-robot.sh`**

```bash
nano ~/stop-robot.sh
```

**Copy and paste this:**

```bash
#!/bin/bash
INSTANCE_ID="i-0f7c599a52fac7e91"

echo "🛑 Stopping EC2 instance..."
aws ec2 stop-instances --instance-ids $INSTANCE_ID

echo "⏳ Waiting 30 seconds..."
sleep 30

echo ""
echo "✅ Instance stopped!"
echo "💰 Saving ~\$15/month while stopped!"
```

**Make it executable:**
```bash
chmod +x ~/stop-robot.sh
```

---

#### **C) Create `check-robot.sh`**

```bash
nano ~/check-robot.sh
```

**Copy and paste this:**

```bash
#!/bin/bash
INSTANCE_ID="i-0f7c599a52fac7e91"

echo "📊 Robot Backend Status"
echo "======================="
echo ""

STATUS=$(aws ec2 describe-instances --instance-ids $INSTANCE_ID \
  --query 'Reservations[0].Instances[0].State.Name' \
  --output text)

IP=$(aws ec2 describe-instances --instance-ids $INSTANCE_ID \
  --query 'Reservations[0].Instances[0].PublicIpAddress' \
  --output text 2>/dev/null)

echo "Instance ID: $INSTANCE_ID"
echo "Status: $STATUS"

if [ "$STATUS" = "running" ]; then
  echo "Public IP: $IP"
  echo "Backend URL: http://$IP"
  echo ""
  echo "✅ Instance is RUNNING"
  echo ""
  echo "Test: curl http://$IP/health -H \"Authorization: Bearer \$(cat ~/Documents/P2RemoteConnection/token.txt)\""
elif [ "$STATUS" = "stopped" ]; then
  echo "Public IP: N/A (stopped)"
  echo ""
  echo "⚠️  Instance is STOPPED"
  echo "💰 Currently saving money!"
  echo ""
  echo "To start: ~/start-robot.sh"
else
  echo ""
  echo "ℹ️  Instance status: $STATUS"
fi
```

**Make it executable:**
```bash
chmod +x ~/check-robot.sh
```

---

## 🎯 Daily Usage

### **Start the Robot Backend**

```bash
~/start-robot.sh
```

**Wait for it to show the IP address (about 60 seconds)**

---

### **Check If Running**

```bash
~/check-robot.sh
```

---

### **Test Backend (Replace IP with current one)**

```bash
curl http://YOUR_EC2_IP/health \
  -H "Authorization: Bearer $(cat ~/Documents/P2RemoteConnection/token.txt)"
```

**Expected:** `{"status":"ok"}`

---

### **Start Frontend**

```bash
cd ~/Documents/P2RemoteConnection/src/p2_remote_connection/app
python3 -m http.server 8080
```

**Open browser:** http://localhost:8080/go2_joystick.html

**Login with password from `token.txt`**

---

### **Stop Backend (IMPORTANT - Saves Money!)**

```bash
~/stop-robot.sh
```

**Always stop when done to save ~$15/month!**

---

## 🔄 Update Backend Code (When Robot Team Changes Code)

### **Step-by-Step Update Process:**

```bash
# 1. Get current EC2 IP
~/check-robot.sh
# Note the IP (e.g., 54.66.38.113)

# 2. Go to your project folder
cd ~/Documents/P2RemoteConnection

# 3. Upload the updated code to EC2
scp -i aws/go2-robot-control-key.pem -r src ubuntu@YOUR_EC2_IP:/opt/go2-backend/

# 4. Restart the backend service
ssh -i aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP \
  "sudo systemctl restart go2-backend"

# 5. Check if backend is running
ssh -i aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP \
  "sudo systemctl status go2-backend"
```

**Example (with actual IP 54.66.38.113):**

```bash
# Upload code
cd ~/Documents/P2RemoteConnection
scp -i aws/go2-robot-control-key.pem -r src ubuntu@54.66.38.113:/opt/go2-backend/

# Restart service
ssh -i aws/go2-robot-control-key.pem ubuntu@54.66.38.113 \
  "sudo systemctl restart go2-backend"
```

**Done! ✅ Backend updated.**

---

## 🆘 Troubleshooting

### **Problem: AWS CLI not configured**

```bash
aws configure
```

Enter your AWS credentials (ask AWS admin if you don't have them)

---

### **Problem: Backend not responding**

```bash
# 1. Check EC2 is running
~/check-robot.sh

# 2. SSH into EC2
ssh -i ~/Documents/P2RemoteConnection/aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP

# 3. Check backend service
sudo systemctl status go2-backend

# 4. View logs
sudo journalctl -u go2-backend -n 50

# 5. Restart if needed
sudo systemctl restart go2-backend

# 6. Exit
exit
```

---

### **Problem: Frontend can't connect**

**The IP changes every time you stop/start!**

1. Get current IP:
   ```bash
   ~/check-robot.sh
   ```

2. Test backend with new IP:
   ```bash
   curl http://NEW_IP/health -H "Authorization: Bearer $(cat ~/Documents/P2RemoteConnection/token.txt)"
   ```

3. Clear browser cache:
   - Open browser Dev Tools (F12)
   - Go to Console tab
   - Type: `localStorage.clear(); location.reload()`

---

### **Problem: Can't SSH**

```bash
# Fix key permissions
chmod 400 ~/Documents/P2RemoteConnection/aws/go2-robot-control-key.pem

# Verify instance is running
~/check-robot.sh

# Use correct IP (from check-robot.sh)
ssh -i ~/Documents/P2RemoteConnection/aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP
```

---

## 📋 All Important Commands

```bash
# ========================================
# START/STOP
# ========================================

# Start EC2
~/start-robot.sh

# Stop EC2 (save money!)
~/stop-robot.sh

# Check status
~/check-robot.sh

# ========================================
# FRONTEND
# ========================================

# Start frontend server
cd ~/Documents/P2RemoteConnection/src/p2_remote_connection/app
python3 -m http.server 8080

# Open: http://localhost:8080/go2_joystick.html
# Password: from token.txt

# ========================================
# TEST BACKEND
# ========================================

# Get current IP first
~/check-robot.sh

# Test health
curl http://YOUR_EC2_IP/health \
  -H "Authorization: Bearer $(cat ~/Documents/P2RemoteConnection/token.txt)"

# ========================================
# SSH INTO EC2
# ========================================

# Connect
ssh -i ~/Documents/P2RemoteConnection/aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP

# Check backend status
sudo systemctl status go2-backend

# View logs
sudo journalctl -u go2-backend -n 50

# View live logs (Ctrl+C to exit)
sudo journalctl -u go2-backend -f

# Restart backend
sudo systemctl restart go2-backend

# ========================================
# UPDATE BACKEND CODE
# ========================================

# Upload updated code from your computer
cd ~/Documents/P2RemoteConnection
scp -i aws/go2-robot-control-key.pem -r src ubuntu@YOUR_EC2_IP:/opt/go2-backend/

# Restart backend service
ssh -i ~/Documents/P2RemoteConnection/aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP \
  "sudo systemctl restart go2-backend"

# ========================================
# AWS CLI COMMANDS
# ========================================

# Start instance
aws ec2 start-instances --instance-ids i-0f7c599a52fac7e91

# Stop instance
aws ec2 stop-instances --instance-ids i-0f7c599a52fac7e91

# Get IP
aws ec2 describe-instances --instance-ids i-0f7c599a52fac7e91 \
  --query 'Reservations[0].Instances[0].PublicIpAddress' \
  --output text

# Get status
aws ec2 describe-instances --instance-ids i-0f7c599a52fac7e91 \
  --query 'Reservations[0].Instances[0].State.Name' \
  --output text
```

---

## 💰 Cost Information

| Status | Monthly Cost |
|--------|--------------|
| **Running 24/7** | ~$18/month |
| **Stopped** | ~$3/month (storage only) |

**💡 TIP:** Always stop the instance when not using it!

```bash
# Stop to save money
~/stop-robot.sh
```

---

## 🔐 Security - Keep These Secret!

- ❌ **Never share** `token.txt` (API password)
- ❌ **Never share** `aws/go2-robot-control-key.pem` (SSH key)
- ❌ **Never share** AWS credentials
- ❌ **Never commit these to Git**

---

## 📞 Quick Reference

| What | Command |
|------|---------|
| **Start robot** | `~/start-robot.sh` |
| **Stop robot** | `~/stop-robot.sh` |
| **Check status** | `~/check-robot.sh` |
| **SSH to EC2** | `ssh -i ~/Documents/P2RemoteConnection/aws/go2-robot-control-key.pem ubuntu@YOUR_EC2_IP` |
| **Start frontend** | `cd ~/Documents/P2RemoteConnection/src/p2_remote_connection/app && python3 -m http.server 8080` |
| **Frontend URL** | http://localhost:8080/go2_joystick.html |
| **Your password** | In `token.txt` file |

---

## ✅ Complete Workflow Example

**Starting your day:**

```bash
# 1. Start EC2
~/start-robot.sh
# Note the IP address shown (e.g., 54.66.38.113)

# 2. Test backend (use IP from step 1)
curl http://54.66.38.113/health \
  -H "Authorization: Bearer $(cat ~/Documents/P2RemoteConnection/token.txt)"
# Should show: {"status":"ok"}

# 3. Start frontend
cd ~/Documents/P2RemoteConnection/src/p2_remote_connection/app
python3 -m http.server 8080

# 4. Open browser
# Go to: http://localhost:8080/go2_joystick.html
# Enter password from token.txt
# Click "Unlock"
# Use robot!
```

**Ending your day:**

```bash
# Stop EC2 to save money
~/stop-robot.sh
```

---

## 🎓 Summary

**You have 3 simple scripts:**
- `~/start-robot.sh` - Start EC2 backend
- `~/stop-robot.sh` - Stop EC2 backend (saves money!)
- `~/check-robot.sh` - Check if running

**Daily workflow:**
1. Run `~/start-robot.sh`
2. Wait for IP address
3. Start frontend: `cd ~/Documents/P2RemoteConnection/src/p2_remote_connection/app && python3 -m http.server 8080`
4. Open: http://localhost:8080/go2_joystick.html
5. When done: `~/stop-robot.sh`

**That's it!** 🚀

---

**Instance ID:** `i-0f7c599a52fac7e91`
**Region:** ap-southeast-2 (Sydney)
**Last Updated:** January 24, 2026
