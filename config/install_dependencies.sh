#!/bin/bash
set -e

YAML_FILE="dependencies.yaml"

if [ ! -f "$YAML_FILE" ]; then
    echo "Error: $YAML_FILE not found."
    exit 1
fi

install_apt() {
    echo "Parsing apt packages..."
    # Extract lines between 'apt:' and the next top-level key (or end of file)
    # Filter for list items, remove hyphens, whitespace, and comments
    PACKAGES=$(sed -n '/^apt:/,/^[a-z]/p' "$YAML_FILE" | grep '^\s*-\s' | sed 's/^\s*-\s*//' | sed 's/#.*//')
    
    if [ -n "$PACKAGES" ]; then
        # Replace newlines with spaces
        PACKAGES=$(echo "$PACKAGES" | tr '\n' ' ')
        echo "Installing apt packages: $PACKAGES"
        apt-get update
        # shellcheck disable=SC2086
        apt-get install -y $PACKAGES
        rm -rf /var/lib/apt/lists/*
    fi
}

install_git() {
    echo "Parsing git repositories..."
    
    # Read the file line by line
    IN_GIT_SECTION=false
    CURRENT_URL=""
    CURRENT_VERSION=""
    CURRENT_PATH=""

    process_repo() {
        if [ -n "$CURRENT_URL" ] && [ -n "$CURRENT_PATH" ]; then
            VERSION=${CURRENT_VERSION:-main}
            FULL_PATH="$(pwd)/$CURRENT_PATH"
            echo "Cloning $CURRENT_URL ($VERSION) to $FULL_PATH"
            
            if [ -d "$FULL_PATH" ]; then
                echo "Path $FULL_PATH already exists, skipping."
            else
                if ! command -v git &> /dev/null; then
                    apt-get update && apt-get install -y git
                fi
                git clone -b "$VERSION" "$CURRENT_URL" "$FULL_PATH"
            fi
        fi
        # Reset variables
        CURRENT_URL=""
        CURRENT_VERSION=""
        CURRENT_PATH=""
    }

    while IFS= read -r line || [[ -n "$line" ]]; do
        # Remove comments
        line=$(echo "$line" | sed 's/#.*//')
        # Skip empty lines
        if [[ -z "$(echo "$line" | xargs)" ]]; then
            continue
        fi

        # Detect top-level keys
        if [[ "$line" =~ ^[a-z].*: ]]; then
            if [[ "$line" =~ ^git: ]]; then
                IN_GIT_SECTION=true
            else
                if [ "$IN_GIT_SECTION" = true ]; then
                    process_repo # Process last repo if any
                    IN_GIT_SECTION=false
                fi
            fi
            continue
        fi

        if [ "$IN_GIT_SECTION" = true ]; then
            # Check for new list item (start of a repo entry)
            if [[ "$line" =~ -\ url: ]]; then
                process_repo # Process previous repo
                CURRENT_URL=$(echo "$line" | sed 's/.*url:\s*//' | xargs)
            elif [[ "$line" =~ url: ]]; then
                 # Handle case where url is not on the dash line (unlikely in valid yaml list but possible)
                 # For simplicity, assume standard format: - url: ...
                 :
            fi
            
            if [[ "$line" =~ version: ]]; then
                CURRENT_VERSION=$(echo "$line" | sed 's/.*version:\s*//' | xargs)
            fi
            
            if [[ "$line" =~ path: ]]; then
                CURRENT_PATH=$(echo "$line" | sed 's/.*path:\s*//' | xargs)
            fi
        fi
    done < "$YAML_FILE"
    
    # Process the last repo if we ended inside the git section
    if [ "$IN_GIT_SECTION" = true ]; then
        process_repo
    fi
}

install_apt
# install_git
