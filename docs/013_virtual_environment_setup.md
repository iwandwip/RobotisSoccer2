# 013 - Virtual Environment Setup Guide

## Overview
This guide explains how to set up Python virtual environments on Ubuntu/Debian systems, specifically addressing the `python3-venv` package requirement and PEP 668 "externally-managed-environment" issues.

## The Problem
Starting with Python 3.11+, many Linux distributions implement PEP 668 which prevents direct pip installations to protect system Python. Additionally, Ubuntu/Debian separates the `venv` module into a separate package.

**Common Error Messages:**
```
error: externally-managed-environment
× This environment is externally managed
```

```
The virtual environment was not created successfully because ensurepip is not available.
On Debian/Ubuntu systems, you need to install the python3-venv package
```

## Solution: Install python3-venv Package

### Step 1: Install Required Packages

For **Python 3.12** (adjust version as needed):
```bash
sudo apt update
sudo apt install python3.12-venv python3-full -y
```

For **generic Python 3**:
```bash
sudo apt update
sudo apt install python3-venv python3-full -y
```

**Package explanations:**
- `python3.12-venv`: Provides venv module for virtual environments
- `python3-full`: Includes additional Python components

### Step 2: Create Virtual Environment

```bash
# Navigate to your project directory
cd ~/robotis_ws

# Create virtual environment
python3 -m venv venv

# Alternative with specific name
python3 -m venv myproject_env
```

### Step 3: Activate Virtual Environment

```bash
# Activate virtual environment
source venv/bin/activate

# Your prompt should change to show (venv)
# Example: (venv) iwandwp@antimysticc:~/robotis_ws$
```

### Step 4: Verify Virtual Environment

```bash
# Check Python location (should be in venv)
which python
# Output: /home/iwandwp/robotis_ws/venv/bin/python

# Check pip location (should be in venv)
which pip
# Output: /home/iwandwp/robotis_ws/venv/bin/pip

# Check Python version
python --version
```

### Step 5: Install Packages

Now you can install packages normally without `--user` or `--break-system-packages`:

```bash
# Install inquirer
pip install inquirer

# Install multiple packages
pip install inquirer requests flask

# Install from requirements file
pip install -r requirements.txt

# Install specific version
pip install inquirer==2.10.1
```

### Step 6: Deactivate Virtual Environment

```bash
# Deactivate when done
deactivate

# Your prompt returns to normal
# Example: iwandwp@antimysticc:~/robotis_ws$
```

## Complete Workflow Example

Here's a complete example for installing inquirer:

```bash
# 1. Install venv package (one-time setup)
sudo apt update
sudo apt install python3.12-venv python3-full -y

# 2. Create virtual environment
cd ~/robotis_ws
python3 -m venv venv

# 3. Activate virtual environment
source venv/bin/activate

# 4. Upgrade pip (recommended)
pip install --upgrade pip

# 5. Install inquirer
pip install inquirer

# 6. Test installation
python -c "import inquirer; print('Inquirer installed successfully!')"

# 7. Create requirements.txt for future use
pip freeze > requirements.txt

# 8. Deactivate when done
deactivate
```

## Managing Multiple Virtual Environments

### Naming Conventions
```bash
# Project-specific environments
python3 -m venv robotis_env
python3 -m venv web_scraper_env
python3 -m venv data_analysis_env

# Version-specific environments
python3 -m venv py312_env
python3 -m venv django_env
```

### Environment Organization
```bash
# Option 1: In project directory
~/robotis_ws/venv/

# Option 2: Centralized environments
~/venvs/robotis_env/
~/venvs/scraper_env/
~/venvs/analysis_env/
```

### Switching Between Environments
```bash
# Deactivate current environment
deactivate

# Activate different environment
source ~/venvs/other_project_env/bin/activate
```

## Best Practices

### 1. Always Use Virtual Environments
```bash
# ✅ Good: Use virtual environment
source venv/bin/activate
pip install package_name

# ❌ Bad: System-wide installation (will fail with PEP 668)
pip install package_name
```

### 2. Create Requirements File
```bash
# Create requirements.txt
pip freeze > requirements.txt

# Install from requirements.txt (in new environment)
pip install -r requirements.txt
```

### 3. Keep Virtual Environments Clean
```bash
# List installed packages
pip list

# Remove unnecessary packages
pip uninstall package_name

# Create clean environment from requirements
python3 -m venv clean_env
source clean_env/bin/activate
pip install -r requirements.txt
```

### 4. Add Virtual Environment to .gitignore
Virtual environments should NOT be committed to git:
```gitignore
# Virtual environments
venv/
env/
.venv/
*_env/
```

## Common Issues and Solutions

### Issue 1: "ensurepip is not available"
**Solution:**
```bash
sudo apt install python3.12-venv python3-full
```

### Issue 2: Virtual environment not activating
**Symptoms:** Prompt doesn't change, `which python` shows system Python
**Solution:**
```bash
# Check if activation script exists
ls venv/bin/activate

# Use full path
source ~/robotis_ws/venv/bin/activate

# Check shell
echo $SHELL
```

### Issue 3: pip not found in virtual environment
**Solution:**
```bash
# Recreate virtual environment
rm -rf venv
python3 -m venv venv
source venv/bin/activate

# Upgrade pip
python -m pip install --upgrade pip
```

### Issue 4: Packages not found after activation
**Solution:**
```bash
# Verify you're in the right environment
which python
which pip

# Check installed packages
pip list

# Reinstall packages if needed
pip install -r requirements.txt
```

### Issue 5: Permission denied errors
**Solution:**
```bash
# Make sure you're in virtual environment
source venv/bin/activate

# If still issues, recreate environment
deactivate
rm -rf venv
python3 -m venv venv
source venv/bin/activate
```

## Troubleshooting Commands

### Environment Diagnostics
```bash
# Check if in virtual environment
echo $VIRTUAL_ENV

# Check Python executable location
which python
python -c "import sys; print(sys.executable)"

# Check site-packages directory
python -c "import site; print(site.getsitepackages())"

# List all Python paths
python -c "import sys; print('\n'.join(sys.path))"
```

### System Information
```bash
# Check Python version
python3 --version

# Check available Python packages
apt list --installed | grep python3

# Check if venv module is available
python3 -c "import venv; print('venv module available')"
```

## Integration with IDEs

### VS Code
```bash
# After creating virtual environment
# Open VS Code in project directory
code .

# VS Code should detect venv automatically
# Or manually select interpreter:
# Ctrl+Shift+P -> "Python: Select Interpreter" -> choose venv/bin/python
```

### PyCharm
1. File → Settings → Project → Python Interpreter
2. Add Interpreter → Existing Environment
3. Select `venv/bin/python`

## Advanced Tips

### 1. Automatic Environment Activation
Add to your `.bashrc` for automatic activation:
```bash
# Auto-activate venv when entering directory
cd_and_activate() {
    cd "$@"
    if [[ -f "venv/bin/activate" ]]; then
        source venv/bin/activate
    fi
}
alias cd=cd_and_activate
```

### 2. Quick Environment Creation Script
Create `create_venv.sh`:
```bash
#!/bin/bash
ENV_NAME=${1:-venv}
python3 -m venv $ENV_NAME
source $ENV_NAME/bin/activate
pip install --upgrade pip
echo "Virtual environment '$ENV_NAME' created and activated"
echo "To deactivate, run: deactivate"
```

### 3. Environment Information Display
Add to your prompt to show active environment:
```bash
# Add to ~/.bashrc
export PS1="(\$VIRTUAL_ENV) \u@\h:\w\$ "
```

## Summary

1. **Install python3-venv**: `sudo apt install python3.12-venv python3-full`
2. **Create environment**: `python3 -m venv venv`
3. **Activate**: `source venv/bin/activate`
4. **Install packages**: `pip install package_name`
5. **Deactivate**: `deactivate`

Virtual environments solve both PEP 668 "externally-managed-environment" issues and provide project isolation. Always use them for Python development!