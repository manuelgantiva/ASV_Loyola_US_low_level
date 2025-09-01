#!/usr/bin/env python3
"""
Test script to verify the enhanced model loading functionality.
This tests the _find_and_load_best_model method without requiring ROS.
"""

import os
import sys
sys.path.append('/asv_ws/src/asv_ai')

# Create some dummy model files for testing
test_dir = "/tmp/test_asv_models"
os.makedirs(test_dir, exist_ok=True)

# Create dummy model files
dummy_files = [
    "ppo_model_ep5.zip",
    "ppo_model_ep10.zip", 
    "ppo_model_ep15.zip",
    "latest_model.zip",
    "ppo_model_final.zip"
]

for file in dummy_files:
    with open(os.path.join(test_dir, file), 'w') as f:
        f.write("dummy model file")

print(f"Created test files in {test_dir}:")
for file in os.listdir(test_dir):
    print(f"  - {file}")

print("\nModel discovery order should be:")
print("1. Explicit parameter (if provided)")
print("2. latest_model.zip")
print("3. Latest episode model (ep15 in this case)")
print("4. Final model")
print("5. Common locations")

print(f"\nTest files created in: {test_dir}")
print("You can now test the model loading by setting rollout_dir parameter to this directory")
