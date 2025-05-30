document.addEventListener('DOMContentLoaded', function() {
    const logWorkoutForm = document.getElementById('log-workout-form');
    const logTypeSelect = document.getElementById('log-type');
    const routineSelectionArea = document.getElementById('routine-selection-area');
    const routineSelect = document.getElementById('routine-select');
    const exerciseSelectionArea = document.getElementById('exercise-selection-area');
    const exerciseSelect = document.getElementById('exercise-select'); // Dropdown for ad-hoc exercises
    const logWorkoutMessage = document.getElementById('log-workout-message');
    
    // Form fields for performance details
    const weightLiftedInput = document.getElementById('weight_lifted_kg');
    const performedSetsInput = document.getElementById('performed_sets');
    const performedRepsInput = document.getElementById('performed_reps');

    let nextWorkoutData = null; // To store data from sessionStorage

    // Try to load next workout data from session storage
    function loadNextWorkoutFromStorage() {
        const storedData = sessionStorage.getItem('nextWorkoutDetails');
        if (storedData) {
            nextWorkoutData = JSON.parse(storedData);
        }
    }

    // Populate routines dropdown
    async function loadRoutines() {
        if (!routineSelect) return;
        try {
            const routines = await fetchData('/routines');
            routineSelect.innerHTML = '<option value="">-- Select Routine --</option>';
            routines.forEach(routine => {
                const option = document.createElement('option');
                option.value = routine.id;
                option.textContent = routine.name;
                routineSelect.appendChild(option);
            });
        } catch (error) {
            console.error('Error loading routines:', error);
            if (logWorkoutMessage) displayMessage('log-workout-message','Error loading routines.', true);
        }
    }

    // Populate exercises dropdown
    async function loadExercises() {
        if (!exerciseSelect) return;
        try {
            const exercises = await fetchData('/exercises');
            exerciseSelect.innerHTML = '<option value="">-- Select Exercise --</option>';
            exercises.forEach(exercise => {
                const option = document.createElement('option');
                option.value = exercise.id;
                option.textContent = exercise.name;
                exerciseSelect.appendChild(option);
            });
        } catch (error) {
            console.error('Error loading exercises:', error);
            if (logWorkoutMessage) displayMessage('log-workout-message','Error loading exercises.', true);
        }
    }

    // Toggle visibility based on log type
    if (logTypeSelect) {
        logTypeSelect.addEventListener('change', function() {
            if (this.value === 'routine') {
                routineSelectionArea.style.display = 'block';
                exerciseSelectionArea.style.display = 'none';
                exerciseSelect.value = ''; // Clear selection
            } else { // 'exercise'
                routineSelectionArea.style.display = 'none';
                exerciseSelectionArea.style.display = 'block';
                routineSelect.value = ''; // Clear selection
                prefillWeightForSelectedExercise(); // Attempt to prefill if exercise is selected
            }
        });
    }

    // Add event listener to exerciseSelect to prefill weight when an exercise is chosen
    if (exerciseSelect) {
        exerciseSelect.addEventListener('change', prefillWeightForSelectedExercise);
    }
    
    function prefillFieldsForSelectedExercise() {
        const selectedExerciseId = exerciseSelect.value; // ID from the ad-hoc exercise dropdown
        
        // Clear fields first
        if (weightLiftedInput) weightLiftedInput.value = '';
        if (performedSetsInput) performedSetsInput.value = '';
        if (performedRepsInput) performedRepsInput.value = '';

        if (selectedExerciseId && nextWorkoutData && nextWorkoutData.exercises) {
            const matchedExercise = nextWorkoutData.exercises.find(ex => ex.id == selectedExerciseId);
            if (matchedExercise) {
                // Pre-fill weight
                if (matchedExercise.suggested_weight !== "Bodyweight" && typeof matchedExercise.suggested_weight === 'number') {
                    if (weightLiftedInput) weightLiftedInput.value = matchedExercise.suggested_weight.toFixed(2);
                }
                // Pre-fill target sets
                if (matchedExercise.target_sets !== null && performedSetsInput) {
                    performedSetsInput.value = matchedExercise.target_sets;
                }
                // Pre-fill target reps
                if (performedRepsInput) {
                    if (matchedExercise.target_reps === null) { // AMRAP
                        performedRepsInput.value = 'AMRAP';
                    } else if (matchedExercise.target_reps !== null) {
                        performedRepsInput.value = matchedExercise.target_reps;
                    }
                }
            }
        }
    }


    // Handle Log Workout form submission
    if (logWorkoutForm) {
        logWorkoutForm.addEventListener('submit', async function(event) {
            event.preventDefault();
            const formData = new FormData(logWorkoutForm);
            const data = {};

            // Common fields
            if (formData.get('performed_at')) data.performed_at = formData.get('performed_at');
            
            // New performed sets/reps fields
            const performedSetsVal = formData.get('performed_sets');
            if (performedSetsVal) data.performed_sets = parseInt(performedSetsVal, 10);
            
            const performedRepsVal = formData.get('performed_reps');
            if (performedRepsVal) {
                if (performedRepsVal.toUpperCase() === 'AMRAP' || performedRepsVal === '') {
                    data.performed_reps = null; // Backend expects null for AMRAP if column is INTEGER
                } else {
                    data.performed_reps = parseInt(performedRepsVal, 10);
                }
            }

            if (formData.get('duration_completed_minutes')) data.duration_completed_minutes = parseInt(formData.get('duration_completed_minutes'), 10);
            if (formData.get('weight_lifted_kg') && formData.get('weight_lifted_kg') !== '') { // Ensure not empty string before parseFloat
                 data.weight_lifted_kg = parseFloat(formData.get('weight_lifted_kg'));
            } else {
                 data.weight_lifted_kg = null; // Send null if empty, DB column is REAL nullable
            }
            if (formData.get('notes')) data.notes = formData.get('notes');

            const logType = formData.get('log_type');
            if (logType === 'routine' && formData.get('routine_id')) {
                data.routine_id = parseInt(formData.get('routine_id'), 10);
            } else if (logType === 'exercise' && formData.get('exercise_id')) {
                data.exercise_id = parseInt(formData.get('exercise_id'), 10);
            } else {
                if (logWorkoutMessage) displayMessage('log-workout-message','Please select a routine or an exercise.', true);
                return;
            }
            
            if (!data.routine_id && !data.exercise_id) {
                 if (logWorkoutMessage) displayMessage('log-workout-message','Selection missing for routine or exercise.', true);
                return;
            }

            try {
                const loggedWorkout = await fetchData('/history', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                if (logWorkoutMessage) displayMessage('log-workout-message', 'Workout logged successfully!');
                logWorkoutForm.reset();
                // Reset select visibility to default (routine)
                if(logTypeSelect) logTypeSelect.value = 'routine';
                if(routineSelectionArea) routineSelectionArea.style.display = 'block';
                if(exerciseSelectionArea) exerciseSelectionArea.style.display = 'none';
            } catch (error) {
                console.error('Error logging workout:', error);
                if (logWorkoutMessage) displayMessage('log-workout-message', error.message || 'Error logging workout.', true);
            }
        });
    }

    // Initial setup
    loadNextWorkoutFromStorage(); // Load data first
    loadRoutines();
    loadExercises().then(() => {
        // After exercises are loaded into dropdown, if ad-hoc exercise is default view, try prefilling
        if (logTypeSelect && logTypeSelect.value === 'exercise') {
            prefillFieldsForSelectedExercise(); // Changed from prefillWeight...
        }
    }); 

    // Set initial visibility based on default log type
    if (logTypeSelect && routineSelectionArea && exerciseSelectionArea) {
        if (logTypeSelect.value === 'routine') {
            routineSelectionArea.style.display = 'block';
            exerciseSelectionArea.style.display = 'none';
        } else { // 'exercise'
            routineSelectionArea.style.display = 'none';
            exerciseSelectionArea.style.display = 'block';
            // prefillFieldsForSelectedExercise(); // Done after loadExercises now
        }
    }
});
