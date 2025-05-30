from flask import Blueprint, request, jsonify
from .db import query_db, get_db

bp = Blueprint('api', __name__, url_prefix='/api')

# Helper function to get a single item or 404
def get_or_404(table_name, item_id, message_if_not_found="Item not found"):
    item = query_db(f"SELECT * FROM {table_name} WHERE id = ?", [item_id], one=True)
    if item is None:
        return jsonify({"error": message_if_not_found}), 404
    return dict(item)

# == Exercises Endpoints ==

@bp.route('/exercises', methods=['POST'])
def create_exercise():
    data = request.get_json()
    if not data or not data.get('name'):
        return jsonify({"error": "Exercise name is required."}), 400

    name = data['name']
    description = data.get('description')
    muscle_group = data.get('muscle_group')
    equipment = data.get('equipment')
    current_weight = data.get('current_weight', 0.0) # Default if not provided
    weight_increment = data.get('weight_increment', 2.5) # Default if not provided

    try:
        # Check if exercise already exists
        existing = query_db("SELECT id FROM exercises WHERE name = ?", [name], one=True)
        if existing:
            return jsonify({"error": "Exercise with this name already exists."}), 409 # Conflict

        result, last_row_id = query_db(
            "INSERT INTO exercises (name, description, muscle_group, equipment, current_weight, weight_increment) VALUES (?, ?, ?, ?, ?, ?)",
            [name, description, muscle_group, equipment, float(current_weight), float(weight_increment)]
        )
        if last_row_id:
            created_exercise = query_db("SELECT * FROM exercises WHERE id = ?", [last_row_id], one=True)
            return jsonify(dict(created_exercise)), 201
        else:
            # Fallback if last_row_id is not supported/returned correctly by query_db for some reason
            # This part might need adjustment based on how query_db is implemented or sqlite version
            created_exercise = query_db("SELECT * FROM exercises WHERE name = ?", [name], one=True)
            if created_exercise:
                 return jsonify(dict(created_exercise)), 201
            else:
                 return jsonify({"error": "Failed to create exercise and retrieve it."}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/exercises', methods=['GET'])
def get_exercises():
    try:
        exercises_tuples = query_db("SELECT * FROM exercises")
        exercises_list = [dict(ex) for ex in exercises_tuples]
        return jsonify(exercises_list), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/exercises/<int:exercise_id>', methods=['GET'])
def get_exercise(exercise_id):
    try:
        exercise = get_or_404('exercises', exercise_id, "Exercise not found")
        if isinstance(exercise, tuple): # Checking if it's a response tuple (error, status_code)
            return exercise 
        return jsonify(exercise), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/exercises/<int:exercise_id>', methods=['PUT'])
def update_exercise(exercise_id):
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data provided for update."}), 400

    # Check if exercise exists
    exercise = query_db("SELECT * FROM exercises WHERE id = ?", [exercise_id], one=True)
    if not exercise:
        return jsonify({"error": "Exercise not found."}), 404
    
    # Create a dict from the sqlite3.Row object to update it
    updated_exercise_data = dict(exercise)
    
    # Update fields if they are in the request data
    if 'name' in data:
        updated_exercise_data['name'] = data['name']
    if 'description' in data:
        updated_exercise_data['description'] = data['description']
    if 'muscle_group' in data:
        updated_exercise_data['muscle_group'] = data['muscle_group']
    if 'equipment' in data:
        updated_exercise_data['equipment'] = data['equipment']
    if 'current_weight' in data:
        updated_exercise_data['current_weight'] = float(data['current_weight'])
    if 'weight_increment' in data:
        updated_exercise_data['weight_increment'] = float(data['weight_increment'])

    try:
        # Check for name conflict if name is being changed
        if 'name' in data and data['name'] != exercise['name']:
            existing = query_db("SELECT id FROM exercises WHERE name = ? AND id != ?", [data['name'], exercise_id], one=True)
            if existing:
                return jsonify({"error": "Another exercise with this name already exists."}), 409

        query_db(
            """UPDATE exercises SET 
               name = ?, description = ?, muscle_group = ?, equipment = ?,
               current_weight = ?, weight_increment = ? 
               WHERE id = ?""",
            [
                updated_exercise_data['name'], updated_exercise_data['description'],
                updated_exercise_data['muscle_group'], updated_exercise_data['equipment'],
                updated_exercise_data['current_weight'], updated_exercise_data['weight_increment'],
                exercise_id
            ]
        )
        updated_exercise = query_db("SELECT * FROM exercises WHERE id = ?", [exercise_id], one=True)
        return jsonify(dict(updated_exercise)), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/exercises/<int:exercise_id>', methods=['DELETE'])
def delete_exercise(exercise_id):
    try:
        # Check if exercise exists
        exercise = query_db("SELECT * FROM exercises WHERE id = ?", [exercise_id], one=True)
        if not exercise:
            return jsonify({"error": "Exercise not found."}), 404

        query_db("DELETE FROM exercises WHERE id = ?", [exercise_id])
        # Also delete references in routine_exercises
        query_db("DELETE FROM routine_exercises WHERE exercise_id = ?", [exercise_id])
        # Potentially delete from workout_history if direct exercise logs are common and should be removed
        # query_db("DELETE FROM workout_history WHERE exercise_id = ? AND routine_id IS NULL", [exercise_id])
        return jsonify({"message": "Exercise deleted successfully."}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# == Workout Routines Endpoints ==

@bp.route('/routines', methods=['POST'])
def create_routine():
    data = request.get_json()
    if not data or not data.get('name'):
        return jsonify({"error": "Routine name is required."}), 400

    name = data['name']
    description = data.get('description')

    try:
        existing = query_db("SELECT id FROM workout_routines WHERE name = ?", [name], one=True)
        if existing:
            return jsonify({"error": "Routine with this name already exists."}), 409

        result, last_row_id = query_db(
            "INSERT INTO workout_routines (name, description) VALUES (?, ?)",
            [name, description]
        )
        if last_row_id:
            created_routine = query_db("SELECT * FROM workout_routines WHERE id = ?", [last_row_id], one=True)
            return jsonify(dict(created_routine)), 201
        else:
            # Fallback
            created_routine = query_db("SELECT * FROM workout_routines WHERE name = ?", [name], one=True)
            if created_routine:
                return jsonify(dict(created_routine)), 201
            else:
                return jsonify({"error": "Failed to create routine and retrieve it."}), 500
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/routines', methods=['GET'])
def get_routines():
    try:
        routines_tuples = query_db("SELECT * FROM workout_routines")
        routines_list = [dict(r) for r in routines_tuples]
        return jsonify(routines_list), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/routines/<int:routine_id>', methods=['GET'])
def get_routine(routine_id):
    try:
        routine = query_db("SELECT * FROM workout_routines WHERE id = ?", [routine_id], one=True)
        if not routine:
            return jsonify({"error": "Routine not found."}), 404
        
        routine_dict = dict(routine)
        
        # Fetch associated exercises
        routine_exercises_tuples = query_db("""
            SELECT re.id as routine_exercise_id, e.id as exercise_id, e.name, e.description, e.muscle_group, e.equipment,
                   re.sets, re.reps, re.duration_minutes, re.rest_seconds, re.order_in_routine
            FROM routine_exercises re
            JOIN exercises e ON re.exercise_id = e.id
            WHERE re.routine_id = ?
            ORDER BY re.order_in_routine
        """, [routine_id])
        
        routine_dict['exercises'] = [dict(ex) for ex in routine_exercises_tuples]
        return jsonify(routine_dict), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/routines/<int:routine_id>', methods=['PUT'])
def update_routine(routine_id):
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data provided for update."}), 400

    # Check if routine exists
    routine = query_db("SELECT * FROM workout_routines WHERE id = ?", [routine_id], one=True)
    if not routine:
        return jsonify({"error": "Routine not found."}), 404

    updated_routine_data = dict(routine)
    if 'name' in data:
        updated_routine_data['name'] = data['name']
    if 'description' in data:
        updated_routine_data['description'] = data['description']

    try:
        if 'name' in data and data['name'] != routine['name']:
            existing = query_db("SELECT id FROM workout_routines WHERE name = ? AND id != ?", [data['name'], routine_id], one=True)
            if existing:
                return jsonify({"error": "Another routine with this name already exists."}), 409
        
        query_db(
            "UPDATE workout_routines SET name = ?, description = ? WHERE id = ?",
            [updated_routine_data['name'], updated_routine_data['description'], routine_id]
        )
        updated_routine = query_db("SELECT * FROM workout_routines WHERE id = ?", [routine_id], one=True)
        return jsonify(dict(updated_routine)), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/routines/<int:routine_id>', methods=['DELETE'])
def delete_routine(routine_id):
    try:
        routine = query_db("SELECT * FROM workout_routines WHERE id = ?", [routine_id], one=True)
        if not routine:
            return jsonify({"error": "Routine not found."}), 404

        # ON DELETE CASCADE in schema handles routine_exercises entries
        query_db("DELETE FROM workout_routines WHERE id = ?", [routine_id])
        # Also consider workout_history: should we delete history if a routine is deleted?
        # For now, schema sets routine_id to NULL in workout_history.
        # query_db("DELETE FROM workout_history WHERE routine_id = ?", [routine_id]) # If hard delete is preferred
        return jsonify({"message": "Routine deleted successfully."}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# -- Managing exercises within a routine --
@bp.route('/routines/<int:routine_id>/exercises', methods=['POST'])
def add_exercise_to_routine(routine_id):
    data = request.get_json()
    if not data or not data.get('exercise_id') or data.get('order_in_routine') is None:
        return jsonify({"error": "exercise_id and order_in_routine are required."}), 400

    exercise_id = data['exercise_id']
    order_in_routine = data['order_in_routine']
    # Old fields: sets, reps. New fields: target_sets, target_reps
    target_sets = data.get('target_sets') # Will use DB default if None
    target_reps = data.get('target_reps') # Will use DB default if None
    duration_minutes = data.get('duration_minutes')
    rest_seconds = data.get('rest_seconds')

    try:
        # Check if routine and exercise exist
        routine = query_db("SELECT id FROM workout_routines WHERE id = ?", [routine_id], one=True)
        if not routine:
            return jsonify({"error": "Routine not found."}), 404
        
        exercise = query_db("SELECT id FROM exercises WHERE id = ?", [exercise_id], one=True)
        if not exercise:
            return jsonify({"error": "Exercise not found."}), 404

        # Check if order_in_routine is already taken for this routine
        existing_order = query_db(
            "SELECT id FROM routine_exercises WHERE routine_id = ? AND order_in_routine = ?",
            [routine_id, order_in_routine], one=True
        )
        if existing_order:
            return jsonify({"error": f"Order {order_in_routine} is already taken in this routine."}), 409

        result, last_row_id = query_db(
            """INSERT INTO routine_exercises 
               (routine_id, exercise_id, target_sets, target_reps, duration_minutes, rest_seconds, order_in_routine)
               VALUES (?, ?, ?, ?, ?, ?, ?)""",
            [routine_id, exercise_id, target_sets, target_reps, duration_minutes, rest_seconds, order_in_routine]
        )
        
        if last_row_id:
            added_exercise_detail = query_db("SELECT * FROM routine_exercises WHERE id = ?", [last_row_id], one=True)
            return jsonify(dict(added_exercise_detail)), 201
        else:
            # Fallback, may need adjustment based on query_db
            return jsonify({"message": "Exercise added to routine, but couldn't retrieve the specific link."}), 201

    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/routines/<int:routine_id>/exercises/<int:routine_exercise_id>', methods=['DELETE'])
def remove_exercise_from_routine(routine_id, routine_exercise_id):
    # routine_id is part of the path to be explicit, but routine_exercise_id is globally unique
    try:
        link_entry = query_db("SELECT * FROM routine_exercises WHERE id = ? AND routine_id = ?", [routine_exercise_id, routine_id], one=True)
        if not link_entry:
            return jsonify({"error": "Exercise link not found in this routine or link ID is incorrect."}), 404

        query_db("DELETE FROM routine_exercises WHERE id = ?", [routine_exercise_id])
        return jsonify({"message": "Exercise removed from routine successfully."}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# == Workout History Endpoints ==

@bp.route('/history', methods=['POST'])
def create_workout_history():
    data = request.get_json()
    if not data:
        return jsonify({"error": "No data provided."}), 400

    routine_id = data.get('routine_id')
    exercise_id = data.get('exercise_id')
    performed_at = data.get('performed_at') # Expecting ISO format string, or defaults to CURRENT_TIMESTAMP in DB
    # Old: sets_completed, reps_completed. New: performed_sets, performed_reps
    performed_sets = data.get('performed_sets') 
    performed_reps = data.get('performed_reps') 
    duration_completed_minutes = data.get('duration_completed_minutes')
    weight_lifted_kg = data.get('weight_lifted_kg')
    notes = data.get('notes')

    if not routine_id and not exercise_id:
        return jsonify({"error": "Either routine_id or exercise_id must be provided."}), 400

    # Validate foreign keys if provided
    if routine_id:
        routine = query_db("SELECT id FROM workout_routines WHERE id = ?", [routine_id], one=True)
        if not routine:
            return jsonify({"error": f"Routine with id {routine_id} not found."}), 404
    if exercise_id:
        exercise = query_db("SELECT id FROM exercises WHERE id = ?", [exercise_id], one=True)
        if not exercise:
            return jsonify({"error": f"Exercise with id {exercise_id} not found."}), 404
    
    sql_params = [
        routine_id, exercise_id,
        performed_sets, performed_reps, duration_completed_minutes, # Use new field names
        weight_lifted_kg, notes
    ]
    
    insert_sql = """
        INSERT INTO workout_history 
        (routine_id, exercise_id, performed_sets, performed_reps, 
         duration_completed_minutes, weight_lifted_kg, notes""" # Use new field names
    
    if performed_at:
        insert_sql += ", performed_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
        sql_params.append(performed_at)
    else:
        insert_sql += ") VALUES (?, ?, ?, ?, ?, ?, ?)" # Relies on DB default for performed_at

    try:
        result, last_row_id = query_db(insert_sql, sql_params)
        
        if last_row_id:
            # Update current_weight for the exercise if applicable
            if exercise_id and weight_lifted_kg is not None: # Ensure weight_lifted_kg is explicitly provided
                try:
                    query_db(
                        "UPDATE exercises SET current_weight = ? WHERE id = ?",
                        [weight_lifted_kg, exercise_id]
                    )
                    # Optionally, log this update or handle potential errors
                except Exception as update_e:
                    # Log this error, but don't fail the history creation response
                    print(f"Error updating current_weight for exercise {exercise_id}: {update_e}") 
                    # In a real app, use app.logger.error(...)

            created_history = query_db("SELECT * FROM workout_history WHERE id = ?", [last_row_id], one=True)
            return jsonify(dict(created_history)), 201
        else:
            # This fallback might be tricky if performed_at is auto-generated and not known
            return jsonify({"message": "Workout history created, but could not retrieve the exact entry. Weight update might also be affected if ID not retrieved."}), 201
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/history', methods=['GET'])
def get_workout_history_all():
    try:
        # Potentially add pagination in a real app
        history_tuples = query_db("""
            SELECT 
                h.id, h.performed_at, h.notes,
                h.performed_sets, h.performed_reps, h.duration_completed_minutes, h.weight_lifted_kg,
                r.id as routine_id, r.name as routine_name,
                e.id as exercise_id, e.name as exercise_name
            FROM workout_history h
            LEFT JOIN workout_routines r ON h.routine_id = r.id
            LEFT JOIN exercises e ON h.exercise_id = e.id
            ORDER BY h.performed_at DESC
        """)
        history_list = [dict(h) for h in history_tuples]
        return jsonify(history_list), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@bp.route('/history/<int:history_id>', methods=['GET'])
def get_workout_history_item(history_id):
    try:
        history_item = query_db("""
            SELECT 
                h.id, h.performed_at, h.notes,
                h.sets_completed, h.reps_completed, h.duration_completed_minutes, h.weight_lifted_kg,
                r.id as routine_id, r.name as routine_name,
                e.id as exercise_id, e.name as exercise_name
            FROM workout_history h
            LEFT JOIN workout_routines r ON h.routine_id = r.id
            LEFT JOIN exercises e ON h.exercise_id = e.id
            WHERE h.id = ?
        """, [history_id], one=True)
        
        if not history_item:
            return jsonify({"error": "Workout history item not found."}), 404
        return jsonify(dict(history_item)), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

# Note: Update and Delete for history items can be added if necessary,
# but often history is append-only or deletions are handled carefully (soft delete).
# For now, sticking to Create and Read as per primary requirements.

# == Scheduler Endpoint ==
from . import scheduler_logic # Import the new scheduler logic
from .db import query_db # Make sure query_db is accessible, or pass it

@bp.route('/next-workout', methods=['GET'])
def get_next_workout_route():
    try:
        # The scheduler logic needs access to query_db or similar to fetch history
        # We can pass the function query_db from the current context (db.py)
        # This assumes query_db is imported or accessible in this routes.py file.
        # from .db import query_db # Already imported at the top typically
        
        next_workout_info = scheduler_logic.get_next_scheduled_workout_for_display(query_db)
        return jsonify(next_workout_info), 200
    except Exception as e:
        # Log the exception for debugging
        print(f"Error in /next-workout: {str(e)}") # Or use app.logger
        return jsonify({"error": "Could not determine next workout.", "details": str(e)}), 500
