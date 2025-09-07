# 012 - pip Installation Guide

## Overview
This guide provides multiple methods to install pip (Python package installer) on Linux systems when it's not available by default.

## What is pip?
pip is the standard package manager for Python. It allows you to install and manage Python packages from the Python Package Index (PyPI) and other repositories.

## Installation Methods

### Method 1: Install via apt (Recommended for Ubuntu/Debian)

This is the most straightforward method for Ubuntu, Debian, and similar distributions:

```bash
# Update package list
sudo apt update

# Install pip for Python 3
sudo apt install python3-pip -y

# Verify installation
python3 -m pip --version
```

**Advantages:**
- Managed by system package manager
- Automatic security updates
- Proper integration with system Python

### Method 2: Install via get-pip.py (Universal Method)

This method works on most Linux distributions and doesn't require root access:

```bash
# Download get-pip.py
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py

# Install pip for current user
python3 get-pip.py --user

# Add pip to PATH (add to ~/.bashrc for permanent)
export PATH="$HOME/.local/bin:$PATH"

# Verify installation
python3 -m pip --version

# Clean up
rm get-pip.py
```

**Advantages:**
- No root access required
- Works on most distributions
- Latest pip version

### Method 3: Install via ensurepip (Built-in Method)

Python 3.4+ includes ensurepip module for pip installation:

```bash
# Install pip using ensurepip
python3 -m ensurepip --upgrade --user

# Add to PATH if needed
export PATH="$HOME/.local/bin:$PATH"

# Verify installation
python3 -m pip --version
```

**Advantages:**
- Built into Python
- No external downloads
- No root access required

## Verification

After installation, verify pip works correctly:

```bash
# Check pip version
python3 -m pip --version

# List installed packages
python3 -m pip list

# Install a test package (example: inquirer)
python3 -m pip install inquirer --user
```

## Common Issues and Solutions

### Issue 1: "No module named pip"
**Solution:** Use one of the installation methods above.

### Issue 2: "Permission denied" when installing packages
**Solutions:**
```bash
# Option 1: Use --user flag (recommended)
python3 -m pip install package_name --user

# Option 2: Use virtual environment
python3 -m venv myenv
source myenv/bin/activate
pip install package_name
```

### Issue 3: pip not found in PATH after installation
**Solution:**
```bash
# Add to ~/.bashrc
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

### Issue 4: "externally-managed-environment" error (Python 3.11+)
**Solutions:**
```bash
# Option 1: Use --break-system-packages (not recommended)
python3 -m pip install package_name --break-system-packages

# Option 2: Use virtual environment (recommended)
python3 -m venv venv
source venv/bin/activate
pip install package_name

# Option 3: Use --user flag
python3 -m pip install package_name --user
```

## Best Practices

### 1. Use Virtual Environments
```bash
# Create virtual environment
python3 -m venv project_env

# Activate virtual environment
source project_env/bin/activate

# Install packages
pip install package_name

# Deactivate when done
deactivate
```

### 2. Keep pip Updated
```bash
# Update pip itself
python3 -m pip install --upgrade pip --user
```

### 3. Use Requirements Files
```bash
# Create requirements.txt
echo "inquirer==2.10.1" > requirements.txt

# Install from requirements
python3 -m pip install -r requirements.txt --user
```

## Usage Examples

### Install Common Packages
```bash
# Install inquirer for interactive CLI prompts
python3 -m pip install inquirer --user

# Install requests for HTTP requests
python3 -m pip install requests --user

# Install multiple packages
python3 -m pip install inquirer requests flask --user
```

### Package Management
```bash
# List installed packages
python3 -m pip list

# Show package information
python3 -m pip show package_name

# Uninstall package
python3 -m pip uninstall package_name

# Upgrade package
python3 -m pip install --upgrade package_name --user
```

## System-Specific Notes

### WSL2/Ubuntu
- Method 1 (apt) is recommended
- May need to update package list first

### CentOS/RHEL/Fedora
```bash
# For CentOS/RHEL
sudo yum install python3-pip

# For Fedora
sudo dnf install python3-pip
```

### Arch Linux
```bash
sudo pacman -S python-pip
```

## Troubleshooting Commands

```bash
# Check Python installation
which python3
python3 --version

# Check if pip module exists
python3 -c "import pip; print(pip.__version__)"

# Check Python path
python3 -c "import sys; print(sys.path)"

# Check user site packages directory
python3 -m site --user-site
```

## Summary

1. **For Ubuntu/Debian**: Use `sudo apt install python3-pip`
2. **For other systems**: Use `get-pip.py` method
3. **Always use `--user` flag** when installing packages
4. **Consider virtual environments** for project isolation
5. **Keep pip updated** for security and features

This guide ensures you can install and use pip regardless of your Linux distribution or access level.