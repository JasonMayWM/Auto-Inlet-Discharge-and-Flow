-- Exercises Table: Stores information about individual exercises
CREATE TABLE exercises (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT UNIQUE NOT NULL,
    description TEXT,
    muscle_group TEXT,
    equipment TEXT,
    current_weight REAL DEFAULT 0.0,
    weight_increment REAL DEFAULT 2.5
);

-- Workout Routines Table: Stores information about predefined workout routines
CREATE TABLE workout_routines (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    name TEXT NOT NULL,
    description TEXT
);

-- Routine Exercises Table: Links exercises to routines (Many-to-Many relationship)
-- Defines the structure of a workout routine
CREATE TABLE routine_exercises (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    routine_id INTEGER NOT NULL,
    exercise_id INTEGER NOT NULL,
    target_sets INTEGER DEFAULT 3,
    target_reps INTEGER DEFAULT 5,
    duration_minutes INTEGER, -- For time-based exercises like Plank
    rest_seconds INTEGER,     -- Rest time after this exercise within the routine
    order_in_routine INTEGER NOT NULL, -- Sequence of the exercise in the routine
    FOREIGN KEY (routine_id) REFERENCES workout_routines (id) ON DELETE CASCADE,
    FOREIGN KEY (exercise_id) REFERENCES exercises (id) ON DELETE CASCADE
);

-- Workout History Table: Logs completed workouts or individual exercises
CREATE TABLE workout_history (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    -- Can log a full routine or a single ad-hoc exercise
    routine_id INTEGER, 
    exercise_id INTEGER, 
    performed_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    -- Details of the actual performance
    performed_sets INTEGER,         -- Actual sets completed
    performed_reps INTEGER,         -- Actual reps completed (total or for main sets)
    duration_completed_minutes INTEGER, -- Actual duration for timed exercises or total workout
    weight_lifted_kg REAL,      -- Weight lifted for this specific instance
    notes TEXT,                     -- Any notes for this specific workout session
    FOREIGN KEY (routine_id) REFERENCES workout_routines (id) ON DELETE SET NULL,
    FOREIGN KEY (exercise_id) REFERENCES exercises (id) ON DELETE SET NULL,
    CHECK (routine_id IS NOT NULL OR exercise_id IS NOT NULL) -- Ensure at least one is logged
);

-- Core Exercises for the Application
-- Initial current_weight values are examples; users would update these.
INSERT INTO exercises (name, description, muscle_group, equipment, current_weight, weight_increment) VALUES
('Squat', 'Compound lower body exercise.', 'Quads, Glutes, Hamstrings', 'Barbell', 20.0, 2.5),
('Overhead Press', 'Compound upper body pressing exercise.', 'Shoulders, Triceps', 'Barbell', 10.0, 1.25), 
('Bench Press', 'Compound upper body pressing exercise.', 'Chest, Shoulders, Triceps', 'Barbell', 20.0, 2.5),
('Deadlift', 'Compound full-body pulling exercise.', 'Back, Legs, Glutes, Hamstrings', 'Barbell', 40.0, 5.0), 
('Chin-ups', 'Upper body pulling exercise.', 'Back, Biceps', 'Bodyweight/Pull-up bar', 0.0, 0.0); -- current_weight could be bodyweight, increment 0 if not adding weight.

-- Add a generic "Squat 80%" exercise entry if needed for logging, or handle modification via logic/notes.
-- For simplicity with current structure, we'll rely on the name "Squat 80%" in schedule,
-- and assume the user logs "Squat" but notes the 80% or the system implies it.
-- Alternatively, create a specific exercise:
-- INSERT INTO exercises (name, description, muscle_group, equipment) VALUES
-- ('Squat 80%', 'Squat at 80% of working weight.', 'Quads, Glutes, Hamstrings', 'Barbell');


-- Predefined Workout Routines based on the Week 1 / Week 2 schedule
-- These routine names should match those in `scheduler_logic.py` for history tracking.
INSERT INTO workout_routines (name, description) VALUES
('Week 1: Monday', 'Squat, Overhead Press, Deadlift'),
('Week 1: Wednesday', 'Squat 80%, Bench Press, Deadlift'),
('Week 1: Friday', 'Squat, Overhead Press, Chin-ups'),
('Week 2: Monday', 'Squat, Bench Press, Deadlift'),
('Week 2: Wednesday', 'Squat 80%, Overhead Press, Deadlift'),
('Week 2: Friday', 'Squat, Bench Press, Chin-ups');

-- Routine Exercises for Week 1: Monday
INSERT INTO routine_exercises (routine_id, exercise_id, order_in_routine, target_sets, target_reps) VALUES
((SELECT id from workout_routines WHERE name = 'Week 1: Monday'), (SELECT id from exercises WHERE name = 'Squat'), 1, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 1: Monday'), (SELECT id from exercises WHERE name = 'Overhead Press'), 2, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 1: Monday'), (SELECT id from exercises WHERE name = 'Deadlift'), 3, 1, 5);

-- Routine Exercises for Week 1: Wednesday
INSERT INTO routine_exercises (routine_id, exercise_id, order_in_routine, target_sets, target_reps) VALUES
((SELECT id from workout_routines WHERE name = 'Week 1: Wednesday'), (SELECT id from exercises WHERE name = 'Squat'), 1, 3, 5), -- User should remember 80% or UI should specify
((SELECT id from workout_routines WHERE name = 'Week 1: Wednesday'), (SELECT id from exercises WHERE name = 'Bench Press'), 2, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 1: Wednesday'), (SELECT id from exercises WHERE name = 'Deadlift'), 3, 1, 5);

-- Routine Exercises for Week 1: Friday
INSERT INTO routine_exercises (routine_id, exercise_id, order_in_routine, target_sets, target_reps) VALUES
((SELECT id from workout_routines WHERE name = 'Week 1: Friday'), (SELECT id from exercises WHERE name = 'Squat'), 1, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 1: Friday'), (SELECT id from exercises WHERE name = 'Overhead Press'), 2, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 1: Friday'), (SELECT id from exercises WHERE name = 'Chin-ups'), 3, 3, NULL); -- target_reps NULL for AMRAP or bodyweight max

-- Routine Exercises for Week 2: Monday
INSERT INTO routine_exercises (routine_id, exercise_id, order_in_routine, target_sets, target_reps) VALUES
((SELECT id from workout_routines WHERE name = 'Week 2: Monday'), (SELECT id from exercises WHERE name = 'Squat'), 1, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 2: Monday'), (SELECT id from exercises WHERE name = 'Bench Press'), 2, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 2: Monday'), (SELECT id from exercises WHERE name = 'Deadlift'), 3, 1, 5);

-- Routine Exercises for Week 2: Wednesday
INSERT INTO routine_exercises (routine_id, exercise_id, order_in_routine, target_sets, target_reps) VALUES
((SELECT id from workout_routines WHERE name = 'Week 2: Wednesday'), (SELECT id from exercises WHERE name = 'Squat'), 1, 3, 5), -- User should remember 80%
((SELECT id from workout_routines WHERE name = 'Week 2: Wednesday'), (SELECT id from exercises WHERE name = 'Overhead Press'), 2, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 2: Wednesday'), (SELECT id from exercises WHERE name = 'Deadlift'), 3, 1, 5);

-- Routine Exercises for Week 2: Friday
INSERT INTO routine_exercises (routine_id, exercise_id, order_in_routine, target_sets, target_reps) VALUES
((SELECT id from workout_routines WHERE name = 'Week 2: Friday'), (SELECT id from exercises WHERE name = 'Squat'), 1, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 2: Friday'), (SELECT id from exercises WHERE name = 'Bench Press'), 2, 3, 5),
((SELECT id from workout_routines WHERE name = 'Week 2: Friday'), (SELECT id from exercises WHERE name = 'Chin-ups'), 3, 3, NULL);


-- Example Workout History (Optional, but useful for testing scheduler)
-- INSERT INTO workout_history (routine_id, performed_at, notes) VALUES
-- ((SELECT id from workout_routines WHERE name = 'Week 1: Monday'), '2024-03-18 10:00:00', 'Completed Week 1 Monday.');
