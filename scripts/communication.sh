#!/bin/bash
echo "=== 🧪 Interactive Test Script ==="
sleep 1
echo "Do you want to continue? (yes/no)"
read answer
if [[ "$answer" != "yes" ]]; then
    echo "❌ Operation cancelled."
    exit 1
fi

sleep 1
echo "Enter your password:"
read -s password
if [[ "$password" != "secret" ]]; then
    echo "🔒 Authentication failed."
    exit 1
fi

sleep 1
echo "✅ Access granted. Running task..."
for i in {1..5}; do
    echo "Processing step $i..."
    sleep 1
done

echo "🎉 Test completed successfully!"
exit 0
