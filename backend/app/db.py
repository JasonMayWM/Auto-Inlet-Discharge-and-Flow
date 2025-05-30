import sqlite3
import os

# Determine the base directory of the project
# This assumes db.py is in backend/app/
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATABASE_PATH = os.path.join(BASE_DIR, 'database', 'workout_app.db')
SCHEMA_PATH = os.path.join(BASE_DIR, 'database', 'schema.sql')

def get_db():
    """Establishes a connection to the SQLite database."""
    db = sqlite3.connect(DATABASE_PATH)
    db.row_factory = sqlite3.Row  # Access columns by name
    return db

def close_db(db=None):
    """Closes the database connection."""
    if db is not None:
        db.close()

def init_db():
    """Initializes the database using the schema.sql file."""
    if not os.path.exists(DATABASE_PATH):
        os.makedirs(os.path.dirname(DATABASE_PATH), exist_ok=True)
        print(f"Database directory: {os.path.dirname(DATABASE_PATH)}")
        print(f"Database path: {DATABASE_PATH}")
        print(f"Schema path: {SCHEMA_PATH}")

    db = get_db()
    cursor = db.cursor()
    try:
        with open(SCHEMA_PATH, 'r') as f:
            sql_script = f.read()
        cursor.executescript(sql_script)
        db.commit()
        print("Database initialized successfully.")
    except sqlite3.Error as e:
        print(f"Error initializing database: {e}")
    except FileNotFoundError:
        print(f"Error: Schema file not found at {SCHEMA_PATH}")
    finally:
        close_db(db)

def query_db(query, args=(), one=False):
    """Helper function to query the database."""
    db = get_db()
    cursor = db.execute(query, args)
    results = cursor.fetchall()
    db.commit() # Important for INSERT, UPDATE, DELETE
    last_row_id = cursor.lastrowid
    close_db(db)
    if one:
        return results[0] if results else None
    return (results, last_row_id) if last_row_id is not None else results

if __name__ == '__main__':
    # For testing db.py directly
    print(f"Project Base Directory: {BASE_DIR}")
    print(f"Database will be created at: {DATABASE_PATH}")
    print(f"Schema will be read from: {SCHEMA_PATH}")
    # Check if schema file exists
    if os.path.exists(SCHEMA_PATH):
        print("Schema file found.")
        init_db()
    else:
        print("Schema file not found. Please ensure database/schema.sql exists.")

    # Example usage (after initialization)
    if os.path.exists(DATABASE_PATH):
        print("\nAttempting to query exercises (should be empty or have examples if schema ran)...")
        exercises = query_db("SELECT * FROM exercises")
        if exercises:
            for ex in exercises:
                print(dict(ex))
        else:
            print("No exercises found or table doesn't exist yet.")
