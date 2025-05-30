import os
from backend.app import create_app # Adjusted import path

# The create_app function should be defined in backend/app/__init__.py
# BASE_DIR for db might need to be adjusted if app.py is outside backend/
# However, db.py calculates BASE_DIR from its own location, so it should be fine.

application = create_app()

if __name__ == '__main__':
    # For local development, using Flask's built-in server
    # The host '0.0.0.0' makes it accessible from any IP on the network
    # The FLASK_APP environment variable would typically be set to backend.app:create_app or this file.
    # e.g. FLASK_APP=backend.app:application flask run
    # or python backend/app.py
    
    # Check if the DB exists, if not, offer to initialize it.
    # This is a bit of a simplification; in a real app, migrations (like Alembic) are better.
    db_path_check = os.path.join(os.path.dirname(__file__), '..', 'database', 'workout_app.db')
    if not os.path.exists(db_path_check):
        print("Database not found. To initialize the database, run:")
        print("flask init-db")
        print("Then run the app again: python backend/app.py")
    else:
        print("Starting Flask app...")
        # Make sure to run from the root of the project for `flask init-db` and consistent paths.
        # The app.run() is for local development. For production, a WSGI server like Gunicorn is used.
        application.run(host='0.0.0.0', port=5000, debug=True)
