#!/bin/bash
INSTANCE_ID="i-0f7c599a52fac7e91"
REGION="ap-southeast-2"

echo "🛑 Stopping EC2 instance..."
aws ec2 stop-instances --region "$REGION" --instance-ids "$INSTANCE_ID" >/dev/null

echo "⏳ Waiting for instance to stop..."
aws ec2 wait instance-stopped --region "$REGION" --instance-ids "$INSTANCE_ID"

echo "✅ Instance stopped!"