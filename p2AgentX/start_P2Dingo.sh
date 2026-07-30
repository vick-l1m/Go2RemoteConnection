#!/bin/bash
INSTANCE_ID="i-0f7c599a52fac7e91"
REGION="ap-southeast-2"
PUBLIC_URL="https://p2dingo-control-backend.p2agentx.com"

echo "🚀 Starting EC2 instance..."
aws ec2 start-instances --region "$REGION" --instance-ids "$INSTANCE_ID" >/dev/null

echo "⏳ Waiting for instance to start..."
aws ec2 wait instance-running --region "$REGION" --instance-ids "$INSTANCE_ID"

echo "⏳ Waiting for instance status checks..."
aws ec2 wait instance-status-ok --region "$REGION" --instance-ids "$INSTANCE_ID"

PUBLIC_IP=$(aws ec2 describe-instances --region "$REGION" --instance-ids "$INSTANCE_ID" \
  --query 'Reservations[0].Instances[0].PublicIpAddress' \
  --output text)

echo ""
echo "✅ Instance started!"
echo "📍 Public IP: $PUBLIC_IP"
echo "🌐 Company URL: $PUBLIC_URL"
echo "🧭 Controller page: $PUBLIC_URL/p2dingo"
echo ""
echo "📋 Next steps:"
echo "1. Test backend:"
echo "   curl http://$PUBLIC_IP:8200/health"
echo "   curl $PUBLIC_URL/health"
echo ""
echo "2. Open controller:"
echo "   $PUBLIC_URL/p2dingo"