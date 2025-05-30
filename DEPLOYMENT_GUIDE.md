# Raspberry Pi Deployment Guide for Workout Tracker App

This guide provides step-by-step instructions to deploy the Workout Tracker web application on a Raspberry Pi.

## Table of Contents
1.  [Prerequisites](#1-prerequisites)
2.  [Getting the Application Code](#2-getting-the-application-code)
3.  [Setting up the Backend](#3-setting-up-the-backend)
4.  [Setting up the Frontend (with Nginx)](#4-setting-up-the-frontend-with-nginx)
5.  [Running the Application](#5-running-the-application)
6.  [Troubleshooting Tips](#6-troubleshooting-tips)

---

## 1. Prerequisites

Ensure your Raspberry Pi is set up with Raspberry Pi OS (or a similar Linux distribution) and has network connectivity.

### 1.1. Update System
First, update your system's package list and installed packages:
```bash
sudo apt update
sudo apt upgrade -y
```

### 1.2. Install Python3 and pip
Python3 is usually pre-installed. Verify its installation:
```bash
python3 --version
```
Install `pip` for Python3 if it's not already present:
```bash
sudo apt install python3-pip -y
```

### 1.3. Install git
If you plan to clone the application code from a Git repository:
```bash
sudo apt install git -y
```

### 1.4. Install Production Web Server (Gunicorn)
Flask's built-in development server is not suitable for production. We'll use Gunicorn:
```bash
sudo pip3 install gunicorn
```

### 1.5. Install Nginx (Reverse Proxy)
Nginx will serve our static frontend files and act as a reverse proxy for the Gunicorn/Flask backend:
```bash
sudo apt install nginx -y
```
Enable Nginx to start on boot:
```bash
sudo systemctl enable nginx
sudo systemctl start nginx
```

---

## 2. Getting the Application Code

You need to transfer the application files (the entire project directory including `backend`, `frontend`, `database`, `requirements.txt`, etc.) to your Raspberry Pi. Choose one of the following methods:

### 2.1. Using `git clone` (Recommended if code is in a repository)
Navigate to the directory where you want to store the project (e.g., `/home/pi/`):
```bash
cd /home/pi
git clone <your-repository-url> workout-tracker-app
cd workout-tracker-app
```

### 2.2. Using `scp` (Secure Copy)
From your development machine (not the Pi), navigate to your project directory and run:
```bash
# Replace <pi-user> with your Raspberry Pi username and <pi-ip-address> with its IP address
scp -r . <pi-user>@<pi-ip-address>:/home/<pi-user>/workout-tracker-app
```
Then, SSH into your Raspberry Pi and navigate to the directory:
```bash
ssh <pi-user>@<pi-ip-address>
cd /home/<pi-user>/workout-tracker-app
```
Let's assume the application is now in `/home/pi/workout-tracker-app`.

---

## 3. Setting up the Backend

### 3.1. Create a Virtual Environment
It's good practice to use a virtual environment for Python projects.
```bash
# Ensure python3-venv is installed
sudo apt install python3-venv -y 

# Navigate to your project directory
cd /home/pi/workout-tracker-app

# Create a virtual environment named 'venv'
python3 -m venv venv

# Activate the virtual environment
source venv/bin/activate
```
You should see `(venv)` at the beginning of your command prompt. To deactivate later, simply type `deactivate`.

### 3.2. Install Python Dependencies
Install the required Python libraries using the `requirements.txt` file:
```bash
# Make sure 'venv' is activated
pip install -r requirements.txt
```

### 3.3. Initialize the Database
The application uses an SQLite database. Initialize it using the Flask CLI command.
Ensure your current path is the root of the project (`/home/pi/workout-tracker-app`).
The Flask app is referenced as `backend.app` (meaning the `application` object created by `create_app()` in `backend/app.py` or `backend/app/__init__.py`).

```bash
# Make sure 'venv' is activated
# The FLASK_APP environment variable tells Flask where your app is.
export FLASK_APP=backend.app 
# Or, if backend/app.py is the entry point containing `application = create_app()`:
# export FLASK_APP=backend.app:application

# Run the init-db command registered in backend/app/__init__.py
flask init-db 
```
This command initializes a fresh database based on the `database/schema.sql` script. It will create `database/workout_app.db` if it doesn't exist. 
**Note:** If you run `init-db` on an existing deployment where `workout_app.db` already contains data, this command (as currently implemented) might lead to data loss as it's designed to set up the schema from scratch. For production updates involving schema changes, a proper database migration strategy (e.g., using Alembic) would be necessary but is outside the scope of this basic guide.

Ensure the `database` directory is writable by the user running the command.

### 3.4. Configure Gunicorn to Run Flask App
We will run Gunicorn and have it bind to a Unix socket. This socket will be used by Nginx to communicate with Gunicorn.

Test Gunicorn manually first (from the project root `/home/pi/workout-tracker-app`, with `venv` activated):
```bash
gunicorn --workers 3 --bind unix:workout_tracker.sock -m 007 backend.app:application
```
*   `backend.app:application` assumes your Flask app instance is named `application` and is discoverable via the `backend.app` module (likely from `backend/app.py` after importing `create_app` from `backend.app.__init__`). Adjust if your entry point or app instance variable name is different.
*   `--workers 3`: Adjust based on your Pi model (2-4 is usually fine for a Pi 3/4).
*   `--bind unix:workout_tracker.sock`: Creates a Unix socket file named `workout_tracker.sock` in the current directory.
*   `-m 007`: Sets file permissions for the socket to be world-accessible (less secure, but often needed for Nginx to connect if users differ. A better way is to ensure Gunicorn and Nginx run as users in the same group, e.g., `www-data`). For simplicity now, 007 is used.

Press `Ctrl+C` to stop the manual Gunicorn test.

### 3.5. Setup Systemd Service for Gunicorn
Create a systemd service file to manage the Gunicorn process. This ensures it starts on boot and restarts if it crashes.

Create a file named `workout-tracker.service`:
```bash
sudo nano /etc/systemd/system/workout-tracker.service
```
Paste the following content. **Adjust `User`, `Group`, `WorkingDirectory`, and `ExecStart` paths according to your setup.**

```ini
[Unit]
Description=Gunicorn instance for Workout Tracker App
After=network.target

[Service]
User=pi # Replace pi with the user running the app (e.g., www-data if you prefer)
Group=www-data # Or the group your Nginx worker processes belong to
WorkingDirectory=/home/pi/workout-tracker-app # Path to your project root
Environment="PATH=/home/pi/workout-tracker-app/venv/bin" # Path to venv's bin
Environment="FLASK_APP=backend.app" # Or backend.app:application

ExecStart=/home/pi/workout-tracker-app/venv/bin/gunicorn --workers 3 --bind unix:workout_tracker.sock -m 007 backend.app:application
# If backend.app:application is not found, it might be because of how create_app() is called.
# Ensure backend.app correctly points to the file that creates the Flask 'application' instance.
# The FLASK_APP environment variable might be more reliable here if set correctly.

Restart=always

[Install]
WantedBy=multi-user.target
```

**Explanation of Systemd service file:**
*   `User` and `Group`: The user/group Gunicorn will run as. Ensure this user has permissions to the project files and the socket.
*   `WorkingDirectory`: Set this to your project's root directory.
*   `Environment="PATH=..."`: This makes sure Gunicorn uses the Python from your virtual environment.
*   `ExecStart`: The command to start Gunicorn. It uses the full path to Gunicorn within the virtual environment. The socket file will be created in `WorkingDirectory`.

**Reload systemd, enable, and start the service:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable workout-tracker.service
sudo systemctl start workout-tracker.service
```
Check its status:
```bash
sudo systemctl status workout-tracker.service
```
If it's active and running, Gunicorn is set up.

---

## 4. Setting up the Frontend (with Nginx)

Nginx will serve the static files (HTML, CSS, JS) directly and pass dynamic API requests to Gunicorn.

Create a new Nginx server block configuration file:
```bash
sudo nano /etc/nginx/sites-available/workout-tracker
```
Paste the following configuration, adjusting paths as necessary:

```nginx
server {
    listen 80; # Listen on port 80 (HTTP)
    server_name <your-pi-ip-address> localhost; # Replace with your Pi's IP or a domain name

    # Path to your frontend static files
    root /home/pi/workout-tracker-app/frontend;
    index index.html; # Default file to serve

    # Location for serving static files
    location / {
        try_files $uri $uri/ =404; # Standard try_files for SPA or static sites
    }

    # Location for API requests to be proxied to Gunicorn
    location /api {
        proxy_pass http://unix:/home/pi/workout-tracker-app/workout_tracker.sock; # Path to Gunicorn socket
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    # Optional: You might want to serve other static directories if needed
    # location /static-backend { # If you had static files managed by Flask (not recommended for this setup)
    #     alias /home/pi/workout-tracker-app/backend/app/static; # Example path
    # }
}
```

**Enable this site configuration by creating a symbolic link:**
```bash
sudo ln -s /etc/nginx/sites-available/workout-tracker /etc/nginx/sites-enabled/
```
Remove the default Nginx welcome page if it's linked:
```bash
sudo rm /etc/nginx/sites-enabled/default # Only if it exists and you don't need it
```
Test the Nginx configuration for errors:
```bash
sudo nginx -t
```
If it reports syntax is okay, restart Nginx to apply the changes:
```bash
sudo systemctl restart nginx
```

---

## 5. Running the Application

### 5.1. Start Services
If not already running, start the Gunicorn and Nginx services:
```bash
sudo systemctl start workout-tracker.service
sudo systemctl start nginx
```
Check their status:
```bash
sudo systemctl status workout-tracker.service
sudo systemctl status nginx
```

### 5.2. Find Raspberry Pi's IP Address
You'll need the Pi's IP address to access the application from another device on your network.
```bash
hostname -I
```
This will typically show one or more IP addresses. Use the one relevant to your local network (e.g., starting with `192.168.x.x`).

### 5.3. Access the Application
Open a web browser on a computer or smartphone connected to the same network as your Raspberry Pi. Navigate to:
`http://<your-pi-ip-address>`

You should see the Workout Tracker application's main page.

---

## 6. Troubleshooting Tips

### 6.1. Checking Gunicorn Logs
If the `workout-tracker.service` fails or the backend isn't working:
```bash
sudo journalctl -u workout-tracker.service -e
```
This shows the latest logs from your Gunicorn service. Look for Python errors or Gunicorn errors.
You can also try running the `ExecStart` command directly from the systemd file (while in the `WorkingDirectory` and with `venv` activated) to see errors in the console.

### 6.2. Checking Nginx Logs
If you see Nginx errors (like "502 Bad Gateway") or static files aren't loading:
*   **Access Log:** `/var/log/nginx/access.log`
*   **Error Log:** `/var/log/nginx/error.log`
Check the error log first:
```bash
sudo tail -n 50 /var/log/nginx/error.log
```
A "connect() to unix:/path/to/workout_tracker.sock failed (13: Permission denied)" error in Nginx logs often means Nginx worker processes (usually run as `www-data`) don't have permission to access the Gunicorn socket file created by the Gunicorn user (`pi` in the example).
    *   **Fix 1 (Permissions):** Ensure the `Group` in the systemd service file matches the Nginx user's group (often `www-data`), and that the socket itself is group-writable (Gunicorn's `-m 007` helps, but `-m 070` would be better if users are in the same group).
    *   **Fix 2 (User):** Run Gunicorn as the `www-data` user. This requires `www-data` to have read/execute access to your project directory and write access to the database directory.

### 6.3. Common Permission Issues
*   **Socket Permissions:** As mentioned above, Nginx needs to be able to read/write to the Gunicorn socket.
*   **Database Permissions:** The user running Gunicorn (e.g., `pi` or `www-data`) needs write permissions to the `database/` directory and the `workout_app.db` file. If `init-db` worked but the app can't write, this is a likely cause.
    ```bash
    # Example: If Gunicorn runs as 'pi' and project is in /home/pi/workout-tracker-app
    sudo chown -R pi:pi /home/pi/workout-tracker-app/database
    sudo chmod -R u+w /home/pi/workout-tracker-app/database
    ```
*   **Static File Permissions:** Nginx user (`www-data`) needs read access to the `frontend` directory and its contents. Usually, this is fine if files are world-readable.

### 6.4. Virtual Environment Not Activated for Gunicorn Service
If Gunicorn logs show "ModuleNotFoundError" for Flask or other dependencies, it's likely the systemd service is not using the virtual environment's Python interpreter. Double-check the `Environment="PATH=..."` and `ExecStart` paths in your `.service` file.

### 6.5. FLASK_APP Environment Variable
Ensure `FLASK_APP` is correctly set for Gunicorn to find your Flask application instance. If your `create_app()` function is in `backend/app/__init__.py` and returns the app, and you have an `application = create_app()` in `backend/app.py`, then `backend.app:application` (referring to the variable in `backend/app.py`) is a common way. If `create_app` itself is the factory, Gunicorn can use `backend.app:create_app()`. The systemd service file example uses `backend.app:application`.

---
This guide should provide a solid foundation for deploying your application. Remember to adapt paths and user/group names to your specific Raspberry Pi setup.
```
