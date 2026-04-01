#!/bin/bash
INSTANCE_ID="i-0f7c599a52fac7e91"
REGION="ap-southeast-2"
PUBLIC_URL="https://p2dingo-control-backend.p2agentx.com"

STATUS=$(aws ec2 describe-instances --region "$REGION" --instance-ids "$INSTANCE_ID" \
  --query 'Reservations[0].Instances[0].State.Name' \
  --output text)

echo "📊 P2Dingo Backend Status"
echo "========================="
echo "Instance ID: $INSTANCE_ID"
echo "Region:      $REGION"
echo "Status:      $STATUS"

if [ "$STATUS" = "running" ]; then
  PUBLIC_IP=$(aws ec2 describe-instances --region "$REGION" --instance-ids "$INSTANCE_ID" \
    --query 'Reservations[0].Instances[0].PublicIpAddress' \
    --output text)

  echo "Public IP:   $PUBLIC_IP"
  echo "Company URL: $PUBLIC_URL"
  echo ""
  echo "Quick checks:"
  echo "  curl http://$PUBLIC_IP:8200/health"
  echo "  curl $PUBLIC_URL/health"
else
  echo "Public IP:   N/A"
fi