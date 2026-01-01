#!/bin/bash

# Configuration file
YAML_FILE="config/dependencies.yaml"
WORKSPACE_ROOT="$(pwd)/ros2_ws"

# Check if YAML file exists
if [ ! -f "$YAML_FILE" ]; then
    echo "Error: $YAML_FILE not found."
    exit 1
fi

echo "Reading dependencies from $YAML_FILE..."

# Use python3 to parse YAML and output: url version path
# This avoids needing 'yq' installed, while being robust for YAML parsing.
# We assume python3 and PyYAML are available (verified previously).
DEPENDENCIES=$(python3 -c "
import yaml, sys
try:
    with open('$YAML_FILE', 'r') as f:
        data = yaml.safe_load(f)
        if 'git' in data and data['git']:
            for item in data['git']:
                print(f\"{item['url']} {item['version']} {item['path']}\")
except Exception as e:
    print(f'Error parsing YAML: {e}', file=sys.stderr)
    sys.exit(1)
")

if [ $? -ne 0 ]; then
    echo "Failed to parse YAML file."
    exit 1
fi

if [ -z "$DEPENDENCIES" ]; then
    echo "No git dependencies found."
    exit 0
fi

# Process each dependency
echo "$DEPENDENCIES" | while read -r url version path; do
    if [ -z "$url" ]; then continue; fi

    # Extract repo name from URL (e.g., robot_localization.git -> robot_localization)
    repo_name=$(basename "$url" .git)
    
    # Construct target directory
    target_dir="$WORKSPACE_ROOT/$path/$repo_name"
    
    echo "Processing $repo_name..."
    
    if [ -d "$target_dir" ]; then
        echo "  Directory $target_dir exists. Pulling latest changes..."
        if git -C "$target_dir" pull; then
            echo "  Successfully pulled $repo_name."
        else
            echo "  Error pulling $repo_name."
        fi
    else
        echo "  Cloning $repo_name (branch/tag: $version) into $target_dir..."
        # Create parent directory if it doesn't exist
        mkdir -p "$(dirname "$target_dir")"
        
        if git clone -b "$version" "$url" "$target_dir"; then
            echo "  Successfully cloned $repo_name."
        else
            echo "  Error cloning $repo_name."
        fi
    fi
done

echo "Dependency update complete."
