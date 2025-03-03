#!/bin/bash

#NOTE: There is an issue with this script that will cause it not to work on the first run.
# You must first manually run:
# bloom-generate rosdebian --os-version ubuntu --os-name jammy --ros-distro humble
# and then: 
# fakeroot debian/rules binary
# and ensure the .obj-x86* folder is created. For some reason, that part doesnt work when running this script

# Set workspace path (modify if needed)
WORKSPACE=~/dev/cpp/abv/nora_ws

# Check if a version number was provided as an argument
if [ -z "$1" ]; then
    echo "❌ Error: No version number provided."
    echo "Usage: ./build_nora_debs.sh <version>"
    exit 1
fi

VERSION="$1"
BUILD_NUMBER="0"  # You can manually increment this if needed

echo "🚀 Building Debian packages for version: $VERSION-$BUILD_NUMBER"

# Ensure the workspace exists
if [ ! -d "$WORKSPACE" ]; then
    echo "❌ Error: Workspace directory $WORKSPACE does not exist!"
    exit 1
fi

# Function to update the version number in package.xml
update_package_xml_version() {
    PACKAGE_DIR=$1
    PACKAGE_XML="$PACKAGE_DIR/package.xml"

    if [ -f "$PACKAGE_XML" ]; then
        echo "🔧 Updating package.xml version to $VERSION..."
        sed -i "s|<version>.*</version>|<version>$VERSION</version>|" "$PACKAGE_XML"
    else
        echo "⚠️ Warning: package.xml not found in $PACKAGE_DIR"
    fi
}

# Function to force compat level 12
fix_debian_compat() {
    PACKAGE_DIR=$1
    if [ -f "$PACKAGE_DIR/debian/compat" ]; then
        echo "🔧 Fixing debian/compat for $PACKAGE_DIR..."
        echo "12" > "$PACKAGE_DIR/debian/compat"
    fi
}

# Change to workspace
cd $WORKSPACE

echo "🚀 Building nora_idl..."
colcon build --packages-select nora_idl
if [ $? -ne 0 ]; then
    echo "❌ Error: Failed to build nora_idl"
    exit 1
fi

echo "🛠 Updating package.xml version for nora_idl..."
update_package_xml_version "$WORKSPACE/src/nora_idl"

echo "🛠 Generating Debian package for nora_idl..."
cd $WORKSPACE/src/nora_idl
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble

# ✅ Fix debian/compat and version before running fakeroot
fix_debian_compat "$WORKSPACE/src/nora_idl"

fakeroot debian/rules binary
if [ $? -ne 0 ]; then
    echo "❌ Error: Failed to generate `.deb` for nora_idl"
    exit 1
fi

echo "✅ Installing nora_idl package..."
sudo dpkg -i ../ros-humble-nora-idl_${VERSION}-${BUILD_NUMBER}jammy_amd64.deb
#sudo apt-get install -f -y  # Fix any missing dependencies

cd $WORKSPACE 

echo "🚀 Building nora..."
colcon build --packages-select nora
if [ $? -ne 0 ]; then
    echo "❌ Error: Failed to build nora"
    exit 1
fi

echo "🛠 Updating package.xml version for nora..."
update_package_xml_version "$WORKSPACE/src/nora"

echo "🛠 Generating Debian package for nora..."
cd $WORKSPACE/src/nora
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble

# ✅ Fix debian/compat and version before running fakeroot
fix_debian_compat "$WORKSPACE/src/nora"

fakeroot debian/rules binary
if [ $? -ne 0 ]; then
    echo "❌ Error: Failed to generate `.deb` for nora"
    exit 1
fi

echo "✅ Installing nora package..."
sudo dpkg -i ../ros-humble-nora_${VERSION}-${BUILD_NUMBER}jammy_amd64.deb
#sudo apt-get install -f -y  # Fix any missing dependencies

echo "🎉 All Debian packages built and installed successfully!"
