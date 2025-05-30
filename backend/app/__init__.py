import os
from flask import Flask, jsonify
from flask_cors import CORS # Import CORS

def create_app(test_config=None):
    """Create and configure an instance of the Flask application."""
    app = Flask(__name__, instance_relative_config=True)
    CORS(app) # Enable CORS for all routes and origins by default

    # Ensure the instance folder exists
    try:
        os.makedirs(app.instance_path)
    except OSError:
        pass

    # Database initialization function
    from . import db
    
    @app.cli.command('init-db')
    def init_db_command():
        """Clear existing data and create new tables."""
        db.init_db()
        print('Initialized the database.')

    # Route to initialize DB via HTTP POST request
    @app.route('/init-db', methods=['POST'])
    def init_db_route():
        # Potentially add some security here in a real app (e.g., check for a secret key)
        # For now, allow initialization
        try:
            db.init_db()
            return jsonify({"message": "Database initialized successfully."}), 200
        except Exception as e:
            return jsonify({"error": str(e)}), 500

    # Import and register the routes from routes.py
    from . import routes
    app.register_blueprint(routes.bp) # Assuming routes.py will define a Blueprint named 'bp'

    # A simple hello route
    @app.route('/hello')
    def hello():
        return 'Hello, World!'

    return app
