import json
import os
from datetime import datetime, timedelta

# Assuming this file is in backend/app/
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
WEEK_STATE_FILE = os.path.join(BASE_DIR, 'week_state.json')

# --- Schedule Definition ---
# Using exercise names directly. We can fetch their IDs if needed when presenting to frontend.
# Or, store exercise IDs directly if they are stable and known. For now, names.
WORKOUT_SCHEDULE = {
    1: { # Week 1
        0: {"name": "Week 1: Monday", "exercises": ["Squat", "Overhead Press", "Deadlift"]}, # Monday
        2: {"name": "Week 1: Wednesday", "exercises": ["Squat 80%", "Bench Press", "Deadlift"]},# Wednesday
        4: {"name": "Week 1: Friday", "exercises": ["Squat", "Overhead Press", "Chin-ups"]}  # Friday
    },
    2: { # Week 2
        0: {"name": "Week 2: Monday", "exercises": ["Squat", "Bench Press", "Deadlift"]}, # Monday
        2: {"name": "Week 2: Wednesday", "exercises": ["Squat 80%", "Overhead Press", "Deadlift"]},# Wednesday
        4: {"name": "Week 2: Friday", "exercises": ["Squat", "Bench Press", "Chin-ups"]}  # Friday
    }
}
SCHEDULED_DAYS = sorted(list(WORKOUT_SCHEDULE[1].keys())) # [0, 2, 4] for Mon, Wed, Fri

# --- Week State Management ---
def get_week_state():
    try:
        with open(WEEK_STATE_FILE, 'r') as f:
            return json.load(f)
    except FileNotFoundError:
        # Default state if file doesn't exist
        return {"current_program_week": 1, "last_completed_iso_date": None}

def save_week_state(state):
    with open(WEEK_STATE_FILE, 'w') as f:
        json.dump(state, f, indent=2)

def get_current_program_week():
    return get_week_state().get("current_program_week", 1)

def advance_program_week():
    state = get_week_state()
    current_week = state.get("current_program_week", 1)
    next_week = 2 if current_week == 1 else 1
    state["current_program_week"] = next_week
    # state["last_completed_iso_date"] = datetime.now().date().isoformat() # Could be useful
    save_week_state(state)
    return next_week

# --- Next Workout Logic ---
def get_next_scheduled_workout_for_display(db_query_function):
    """
    Determines the next scheduled workout based on the current program week,
    day of the week, and workout history.
    db_query_function is a function like query_db from db.py to fetch history.
    """
    today = datetime.now()
    today_iso_weekday = today.weekday() # Monday is 0, Sunday is 6
    
    state = get_week_state()
    program_week = state.get("current_program_week", 1)

    # Fetch recent workout history to check for completed workouts
    # We need to identify workouts by a unique name or a combination of routine_id/exercise_id
    # For simplicity, let's assume routine names in WORKOUT_SCHEDULE match routine names in DB
    # if we decide to store these scheduled workouts as routines.
    # For now, we'll check if a workout for a given day in the program week has been logged recently.

    # How far back to check history? Let's say last 7 days to cover the current week.
    # This part is tricky without knowing exactly how workouts are logged (as routines or individual exercises)
    # and how to map them back to the WORKOUT_SCHEDULE.
    
    # Let's assume for now: if a workout is logged with a routine_id, and that routine's name
    # matches one of the "name" fields in WORKOUT_SCHEDULE, it's considered done for this logic.
    
    one_week_ago = (today - timedelta(days=7)).strftime('%Y-%m-%d %H:%M:%S')
    # This query needs to be adapted based on how we identify scheduled workouts in history.
    # For now, let's assume we log a routine whose name matches the schedule's "name"
    recent_workouts_raw = db_query_function(
        """SELECT r.name as routine_name, h.performed_at
           FROM workout_history h
           JOIN workout_routines r ON h.routine_id = r.id
           WHERE h.performed_at >= ?
           ORDER BY h.performed_at DESC""",
        [one_week_ago]
    )
    # Extract unique routine names performed recently
    performed_routine_names = set(dict(row).get('routine_name') for row in recent_workouts_raw if dict(row).get('routine_name'))

    # --- Logic to find next workout ---
    
    # Check current week first
    week_schedule = WORKOUT_SCHEDULE.get(program_week, {})
    
    # Iterate through scheduled days of the current program week (Mon, Wed, Fri)
    for day_index in SCHEDULED_DAYS: # e.g., 0, 2, 4
        if day_index >= today_iso_weekday: # If schedule day is today or later in the week
            workout_details = week_schedule.get(day_index)
            if workout_details and workout_details["name"] not in performed_routine_names:
                return {
                    "program_week": program_week,
                    "day_of_week": day_index, # Monday is 0
                    "workout_name": workout_details["name"],
                    "exercises": _get_exercise_details_for_schedule(workout_details["name"], workout_details["exercises"], db_query_function),
                    "message": "This is your next scheduled workout."
                }
    
    # If all workouts for the current program week (from today onwards) are done,
    # or if it's past the last scheduled day (e.g. it's Saturday/Sunday)
    # then suggest the first workout of the *next* program week.
    
    # Check if ALL workouts for the current program week are done by looking at their names
    all_current_week_workouts_done = True
    for day_idx, details in week_schedule.items():
        if details["name"] not in performed_routine_names:
            all_current_week_workouts_done = False
            break
            
    if all_current_week_workouts_done:
        advanced_week = 2 if program_week == 1 else 1
        # It's important to actually *advance* the week in the state file if we are sure this week is done.
        # This should ideally happen when the *last* workout of the week is completed and logged.
        # For "get_next_workout", we might just predict.
        # Let's assume for now that if we are here, the user is looking ahead,
        # or the current week is truly done. We will advance the week for the suggestion.
        # A more robust system would advance the week upon completion of the Friday workout.
        
        # For this function's purpose, let's just show what *would* be next if the week were advanced.
        # The actual advancement should be a separate trigger.
        # However, the subtask says "If all of the current week's workouts are done, suggest the first workout of the next week (and handle the Week 1/Week 2 alternation)."
        # This implies this function should handle the alternation for suggestion purposes.

        # Let's refine: if it's past Friday, or all workouts this week are done, then we advance for suggestion.
        if today_iso_weekday > SCHEDULED_DAYS[-1] or all_current_week_workouts_done:
            program_week_for_suggestion = 2 if program_week == 1 else 1
            # NOTE: This doesn't *save* the advanced week. That's a separate action.
            # The prompt implies this function should handle the alternation for suggestion.
            # To make this truly useful, perhaps the advance_program_week() should be called by the logging function
            # when a Friday workout is completed.
            # For now, this function will just *show* what's next, which might mean showing next week's stuff.
        else: # Still in current week, but earlier workouts are done, and it's not past Friday yet.
              # This case should have been caught by the loop above.
              # If we reach here, it means all workouts for current week *from today onwards* are done.
              # We should suggest the first workout of the current week if it's not yet done.
              # This makes the logic a bit circular.
              # Let's re-think:
              # 1. Iterate scheduled days in current program week.
              #    If workout on scheduled day is NOT done:
              #        If scheduled day is today or in the future: suggest it.
              #        If scheduled day is in the past (missed workout): suggest it. (POLICY: suggest missed ones first)
              # 2. If all current program week workouts ARE done:
              #    Suggest first workout of NEXT program week. (and the state *should* be advanced elsewhere)

        # Simpler logic:
        # Iterate all scheduled days in current program week. If a workout is not done, suggest it.
        for day_idx in SCHEDULED_DAYS:
            workout_details = week_schedule.get(day_idx)
            if workout_details and workout_details["name"] not in performed_routine_names:
                return {
                    "program_week": program_week,
                    "day_of_week": day_idx,
                    "workout_name": workout_details["name"],
                    "exercises": _get_exercise_details_for_schedule(workout_details["name"], workout_details["exercises"], db_query_function),
                    "message": "Catching up on a workout from this week." if day_idx < today_iso_weekday else "This is your next scheduled workout."
                }

        # If all workouts in the current program week are genuinely done:
        # Then suggest the first workout of the *next* actual program week.
        # This implies the global state of program_week should be advanced when the last workout of a week is logged.
        # For this function, let's assume we need a way to trigger the advance elsewhere.
        # For now, if all current program week workouts are done, then we suggest based on an advanced week.
        
        next_program_week_to_suggest = 2 if program_week == 1 else 1
        next_week_first_day_index = SCHEDULED_DAYS[0]
        next_workout_details = WORKOUT_SCHEDULE[next_program_week_to_suggest][next_week_first_day_index]
        
        return {
            "program_week": next_program_week_to_suggest,
            "day_of_week": next_week_first_day_index,
            "workout_name": next_workout_details["name"],
            "exercises": _get_exercise_details_for_schedule(next_workout_details["name"], next_workout_details["exercises"], db_query_function),
            "message": "All workouts for Program Week {} completed! Starting Program Week {}.".format(program_week, next_program_week_to_suggest)
        }

    # Fallback if no specific workout is found by the logic above (e.g. unexpected state)
    # This part should ideally not be reached if logic is exhaustive for scheduled days.
    return {
        "program_week": program_week,
        "workout_name": "Rest Day / Check Schedule",
        "exercises": [],
        "message": "It's a rest day or your schedule needs review."
    }

def _get_exercise_details_for_schedule(routine_schedule_name, scheduled_exercise_names, db_query_func):
    """
    Fetches details for scheduled exercises, including suggested weight, target_sets, and target_reps.
    routine_schedule_name: The name of the routine from WORKOUT_SCHEDULE (e.g., "Week 1: Monday")
    scheduled_exercise_names: list of strings like ["Squat", "Bench Press 80%"]
    db_query_func: function to query the database.
    """
    detailed_exercises = []
    
    # First, get the routine_id for the given routine_schedule_name
    routine_db_data = db_query_func(
        "SELECT id FROM workout_routines WHERE name = ?",
        [routine_schedule_name],
        one=True
    )
    routine_id = dict(routine_db_data).get("id") if routine_db_data else None

    if not routine_id: # Should not happen if WORKOUT_SCHEDULE names match DB routine names
        for scheduled_name_str in scheduled_exercise_names:
             detailed_exercises.append({
                "original_scheduled_name": scheduled_name_str,
                "name": scheduled_name_str.replace(" 80%", "").strip(),
                "suggested_weight": "N/A - Routine context error",
                "target_sets": "N/A",
                "target_reps": "N/A",
                "is_80_percent": " 80%" in scheduled_name_str
            })
        return detailed_exercises

    for scheduled_name_str in scheduled_exercise_names:
        base_name = scheduled_name_str.replace(" 80%", "").strip()
        is_80_percent = " 80%" in scheduled_name_str

        # Get general exercise data (current_weight, increment)
        exercise_data = db_query_func(
            "SELECT id, name, current_weight, weight_increment FROM exercises WHERE name = ?",
            [base_name],
            one=True
        )

        if exercise_data:
            ex_dict = dict(exercise_data)
            exercise_id = ex_dict["id"]
            current_w = ex_dict.get('current_weight', 0.0) or 0.0
            increment = ex_dict.get('weight_increment', 2.5) or 2.5
            
            suggested_weight_val = "Bodyweight"
            if not (increment == 0.0 and current_w == 0.0): # Not a pure bodyweight exercise
                if increment == 0.0 and current_w > 0.0: # Holding current weight
                    suggested_weight_val = current_w
                else: # Standard progression
                    suggested_weight_val = current_w + increment

            if is_80_percent and isinstance(suggested_weight_val, (int, float)):
                suggested_weight_val = round((suggested_weight_val * 0.8) * 4) / 4
            
            # Get target sets/reps from routine_exercises
            routine_exercise_data = db_query_func(
                "SELECT target_sets, target_reps FROM routine_exercises WHERE routine_id = ? AND exercise_id = ?",
                [routine_id, exercise_id],
                one=True
            )
            
            target_sets_val = dict(routine_exercise_data).get('target_sets', 3) if routine_exercise_data else 3 # Default if not found
            target_reps_val = dict(routine_exercise_data).get('target_reps') if routine_exercise_data else 5 # Default if not found (NULL for AMRAP is fine)


            detailed_exercises.append({
                "id": exercise_id,
                "original_scheduled_name": scheduled_name_str,
                "name": base_name,
                "current_weight": current_w,
                "weight_increment": increment,
                "suggested_weight": suggested_weight_val,
                "target_sets": target_sets_val,
                "target_reps": target_reps_val,
                "is_80_percent": is_80_percent
            })
        else:
            detailed_exercises.append({
                "original_scheduled_name": scheduled_name_str,
                "name": base_name,
                "suggested_weight": "N/A - Exercise not in DB",
                "target_sets": "N/A",
                "target_reps": "N/A",
                "is_80_percent": is_80_percent
            })
    return detailed_exercises


if __name__ == '__main__':
    # Test functions (requires a dummy db.py and schema for query_db to not fail)
    print("Current state:", get_week_state())
    # advance_program_week()
    # print("State after advance:", get_week_state())
    # advance_program_week()
    # print("State after second advance:", get_week_state())

    # Dummy query_db for testing get_next_scheduled_workout_for_display
    def mock_query_db(query, args=(), one=False):
        print(f"Mock DB Query: {query} with {args}")
        if "FROM workout_history" in query:
            # Simulate some completed workouts
            return [
                # {"routine_name": "Week 1: Monday", "performed_at": "2024-03-18 10:00:00"},
                # {"routine_name": "Week 1: Wednesday", "performed_at": "2024-03-20 10:00:00"}
            ]
        return []

    # Simulate today is Monday for consistent testing
    # import datetime
    # class MockDateTime(datetime.datetime):
    #     @classmethod
    #     def now(cls):
    #         return datetime.datetime(2024, 3, 18) # A Monday
    # datetime.datetime = MockDateTime


    print("\nNext workout suggestion:")
    next_workout = get_next_scheduled_workout_for_display(mock_query_db)
    print(json.dumps(next_workout, indent=2))

    # Simulate Monday workout done
    # def mock_query_db_mon_done(query, args=(), one=False):
    #     if "FROM workout_history" in query:
    #         return [{"routine_name": WORKOUT_SCHEDULE[1][0]["name"], "performed_at": "some_date"}]
    #     return []
    # print("\nNext workout suggestion (Mon done):")
    # next_workout = get_next_scheduled_workout_for_display(mock_query_db_mon_done)
    # print(json.dumps(next_workout, indent=2))


    # Simulate Mon, Wed, Fri done for week 1
    # def mock_query_db_all_week1_done(query, args=(), one=False):
    #     if "FROM workout_history" in query:
    #         return [
    #             {"routine_name": WORKOUT_SCHEDULE[1][0]["name"], "performed_at": "some_date"},
    #             {"routine_name": WORKOUT_SCHEDULE[1][2]["name"], "performed_at": "some_date"},
    #             {"routine_name": WORKOUT_SCHEDULE[1][4]["name"], "performed_at": "some_date"},
    #         ]
    #     return []
    # print("\nNext workout suggestion (All week 1 done):")
    # save_week_state({"current_program_week": 1, "last_completed_iso_date": None}) # Ensure week 1
    # next_workout = get_next_scheduled_workout_for_display(mock_query_db_all_week1_done)
    # print(json.dumps(next_workout, indent=2)) # Should suggest Week 2 Monday
    
    # To make this truly testable, we'd need to mock datetime.now() or pass it in.
    # And the logic for advancing program week automatically needs to be clarified:
    # - Does this function advance it? Or just suggest based on advancement?
    # - When is it *actually* advanced and saved? (Likely after logging Friday's workout)

    # The current version of get_next_scheduled_workout_for_display will suggest
    # the next workout of the *next* program week if all current program week's workouts
    # are found in history. It doesn't save the advanced week itself.
    
    # Reset week state for future runs if modified by tests
    # save_week_state({"current_program_week": 1, "last_completed_iso_date": None})
