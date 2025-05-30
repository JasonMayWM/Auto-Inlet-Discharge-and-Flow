document.addEventListener('DOMContentLoaded', function() {
    const addRoutineForm = document.getElementById('add-routine-form');
    const routineList = document.getElementById('routine-list');
    const routineMessage = document.getElementById('routine-message');
    const routineFormTitle = document.querySelector('#add-routine-form-container h3'); // For changing title to "Edit Routine"
    const routineSubmitButton = addRoutineForm.querySelector('button[type="submit"]');

    const routineDetailsContainer = document.getElementById('routine-details-container');
    const selectedRoutineNameDisplay = document.getElementById('selected-routine-name'); // Changed variable name for clarity
    const selectedRoutineExercises = document.getElementById('selected-routine-exercises');
    const addExerciseToRoutineForm = document.getElementById('add-exercise-to-routine-form');
    const exerciseToSelect = document.getElementById('exercise-to-add'); // Dropdown in "Add exercise to routine"
    const routineExerciseMessage = document.getElementById('routine-exercise-message');
    const selectedRoutineIdField = document.getElementById('selected-routine-id'); // Hidden field in "Add exercise to routine" form

    let currentEditingRoutineId = null; // For editing routine's own details (name/desc)
    let currentViewingRoutineId = null; // For viewing/managing exercises of a routine

    // Load all exercises for the "Add exercise to routine" dropdown
    async function loadExercisesForDropdown() {
        if (!exerciseToSelect) return;
        try {
            const exercises = await fetchData('/exercises');
            exerciseToSelect.innerHTML = '<option value="">-- Select Exercise --</option>';
            exercises.forEach(exercise => {
                const option = document.createElement('option');
                option.value = exercise.id;
                option.textContent = exercise.name;
                exerciseToSelect.appendChild(option);
            });
        } catch (error) {
            console.error('Error loading exercises for dropdown:', error);
            if(routineExerciseMessage) displayMessage('routine-exercise-message', 'Could not load exercises for dropdown.', true);
        }
    }

    // Fetch and display all routines
    async function loadRoutines() {
        try {
            const routines = await fetchData('/routines');
            routineList.innerHTML = ''; // Clear loading message
            if (routines.length === 0) {
                routineList.innerHTML = '<li>No routines found. Create one!</li>';
                return;
            }
            routines.forEach(routine => {
                const listItem = document.createElement('li');
                listItem.innerHTML = `
                    <div>
                        <strong>${routine.name}</strong>
                        <p>${routine.description || 'No description.'}</p>
                    </div>
                    <div class="action-buttons">
                        <button class="edit-routine-details" data-id="${routine.id}" data-name="${routine.name}" data-description="${routine.description || ''}">Edit Details</button>
                        <button class="view-routine-exercises" data-id="${routine.id}" data-name="${routine.name}">Manage Exercises</button>
                        <button class="delete-routine" data-id="${routine.id}">Delete Routine</button>
                    </div>
                `;
                listItem.querySelector('.edit-routine-details').addEventListener('click', (e) => populateRoutineFormForEdit(e.target.dataset));
                listItem.querySelector('.view-routine-exercises').addEventListener('click', (e) => showRoutineExercisesManagement(e.target.dataset.id, e.target.dataset.name));
                listItem.querySelector('.delete-routine').addEventListener('click', (e) => deleteRoutine(e.target.dataset.id));
                routineList.appendChild(listItem);
            });
        } catch (error) {
            routineList.innerHTML = '<li>Error loading routines.</li>';
            console.error('Error loading routines:', error);
            if (routineMessage) displayMessage('routine-message', 'Error loading routines.', true);
        }
    }

    // Show exercises management section for a selected routine
    async function showRoutineExercisesManagement(routineId, routineName) {
        currentViewingRoutineId = routineId;
        selectedRoutineIdField.value = routineId; // Set hidden field for the "add exercise to routine" form
        if (selectedRoutineNameDisplay) selectedRoutineNameDisplay.textContent = `Manage Exercises for: ${routineName}`;
        
        try {
            const routine = await fetchData(`/routines/${routineId}`); // This endpoint returns routine details including exercises
            selectedRoutineExercises.innerHTML = ''; 
            if (routine.exercises && routine.exercises.length > 0) {
                routine.exercises.forEach(ex => { // ex here is an item from routine.exercises list
                    const li = document.createElement('li');
                    li.innerHTML = `
                        ${ex.order_in_routine}. ${ex.name} 
                        (Sets: ${ex.target_sets || 'N/A'}, Reps: ${ex.target_reps === null ? 'AMRAP' : (ex.target_reps || 'N/A')}, Duration: ${ex.duration_minutes || 'N/A'} min, Rest: ${ex.rest_seconds || 'N/A'}s)
                        <button class="remove-exercise-from-routine" data-routine-exercise-id="${ex.routine_exercise_id}" data-routine-id="${routineId}">Remove</button>
                    `;
                    li.querySelector('.remove-exercise-from-routine').addEventListener('click', (e) => 
                        removeExerciseFromRoutine(e.target.dataset.routineId, e.target.dataset.routineExerciseId)
                    );
                    selectedRoutineExercises.appendChild(li);
                });
            } else {
                selectedRoutineExercises.innerHTML = '<li>No exercises added to this routine yet.</li>';
            }
            if(routineDetailsContainer) routineDetailsContainer.style.display = 'block'; // Show the section
        } catch (error) {
            console.error(`Error fetching exercises for routine ${routineId}:`, error);
            if(routineExerciseMessage) displayMessage('routine-exercise-message', `Error fetching routine exercises: ${error.message}`, true);
        }
    }
    
    function populateRoutineFormForEdit(routineData) {
        currentEditingRoutineId = routineData.id;
        addRoutineForm.elements['name'].value = routineData.name;
        addRoutineForm.elements['description'].value = routineData.description;
        
        if(routineFormTitle) routineFormTitle.textContent = 'Edit Routine Details';
        if(routineSubmitButton) routineSubmitButton.textContent = 'Update Routine Details';
        
        addRoutineForm.scrollIntoView({ behavior: 'smooth' });
    }

    function resetRoutineForm() {
        addRoutineForm.reset();
        currentEditingRoutineId = null;
        if(routineFormTitle) routineFormTitle.textContent = 'Create New Routine';
        if(routineSubmitButton) routineSubmitButton.textContent = 'Create Routine';
    }

    // Handle Create New Routine / Update Routine Details form submission
    if (addRoutineForm) {
        addRoutineForm.addEventListener('submit', async function(event) {
            event.preventDefault();
            const formData = new FormData(addRoutineForm);
            const data = { // Extract only name and description for this form
                name: formData.get('name'),
                description: formData.get('description')
            };
            
            let method = 'POST';
            let url = '/routines';
            let successMessageAction = 'created';

            if (currentEditingRoutineId) {
                method = 'PUT';
                url = `/routines/${currentEditingRoutineId}`;
                successMessageAction = 'updated';
            }

            try {
                const resultRoutine = await fetchData(url, {
                    method: method,
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                if (routineMessage) displayMessage('routine-message', `Routine "${resultRoutine.name}" ${successMessageAction} successfully!`);
                resetRoutineForm();
                loadRoutines(); // Refresh the list
                // If this was an edit, and the edited routine is currently being viewed for exercise management, update its name display
                if (currentEditingRoutineId && currentViewingRoutineId === currentEditingRoutineId) {
                    if (selectedRoutineNameDisplay) selectedRoutineNameDisplay.textContent = `Manage Exercises for: ${resultRoutine.name}`;
                }

            } catch (error) {
                console.error(`Error ${successMessageAction} routine:`, error);
                if (routineMessage) displayMessage('routine-message', error.message || `Error ${successMessageAction} routine.`, true);
            }
        });
    }
    
    // Handle Add Exercise to Routine form submission
    if (addExerciseToRoutineForm) {
        addExerciseToRoutineForm.addEventListener('submit', async function(event) {
            event.preventDefault();
            const formData = new FormData(addExerciseToRoutineForm);
            const routineIdForExerciseAdd = formData.get('routine_id'); // from hidden input
            
            if (!routineIdForExerciseAdd) { // Should be currentViewingRoutineId
                 if(routineExerciseMessage) displayMessage('routine-exercise-message', 'No routine selected/active for adding an exercise.', true);
                return;
            }

            const targetRepsValue = formData.get('target_reps');
            const exerciseDataForRoutine = {
                exercise_id: formData.get('exercise_id'),
                order_in_routine: parseInt(formData.get('order_in_routine'), 10),
                target_sets: formData.get('target_sets') ? parseInt(formData.get('target_sets'), 10) : null, // Will take DB default if null
                target_reps: (targetRepsValue && targetRepsValue.toUpperCase() !== 'AMRAP' && targetRepsValue !== '') ? parseInt(targetRepsValue, 10) : null,
                duration_minutes: formData.get('duration_minutes') ? parseInt(formData.get('duration_minutes'), 10) : null,
                rest_seconds: formData.get('rest_seconds') ? parseInt(formData.get('rest_seconds'), 10) : null,
            };

            try {
                await fetchData(`/routines/${routineIdForExerciseAdd}/exercises`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                if(routineExerciseMessage) displayMessage('routine-exercise-message', 'Exercise added to routine successfully!');
                addExerciseToRoutineForm.reset(); // Clear form fields
                // Re-set the hidden routine_id as reset clears it
                if(selectedRoutineIdField) selectedRoutineIdField.value = routineIdForExerciseAdd; 
                
                // Refresh the displayed exercises for the current routine
                const currentRoutineName = selectedRoutineNameDisplay.textContent.replace('Manage Exercises for: ', '');
                showRoutineExercisesManagement(routineIdForExerciseAdd, currentRoutineName); 
            } catch (error) {
                console.error('Error adding exercise to routine:', error);
                if(routineExerciseMessage) displayMessage('routine-exercise-message', error.message || 'Error adding exercise to routine.', true);
            }
        });
    }
    // Delete a routine
    async function deleteRoutine(routineId) {
        if (!confirm('Are you sure you want to delete this entire routine? This cannot be undone.')) {
            return;
        }
        try {
            await fetchData(`/routines/${routineId}`, { method: 'DELETE' });
            if (routineMessage) displayMessage('routine-message', 'Routine deleted successfully!');
            loadRoutines(); // Refresh the list of routines
            if (currentViewingRoutineId === routineId) { // If deleted routine's exercises were being viewed
                if(routineDetailsContainer) routineDetailsContainer.style.display = 'none';
                currentViewingRoutineId = null;
            }
            if (currentEditingRoutineId === routineId) { // If deleted routine was being edited
                resetRoutineForm();
            }
        } catch (error) {
            console.error('Error deleting routine:', error);
            if (routineMessage) displayMessage('routine-message', error.message || 'Error deleting routine.', true);
        }
    }
    
    // Remove an exercise from a specific routine
    async function removeExerciseFromRoutine(routineId, routineExerciseId) {
        if (!confirm('Are you sure you want to remove this exercise from the routine?')) {
            return;
        }
        try {
            await fetchData(`/routines/${routineId}/exercises/${routineExerciseId}`, { method: 'DELETE' });
            if (routineExerciseMessage) displayMessage('routine-exercise-message', 'Exercise removed from routine successfully!');
            
            // Refresh the displayed exercises for the current routine
            const currentRoutineName = selectedRoutineNameDisplay.textContent.replace('Manage Exercises for: ', '');
            showRoutineExercisesManagement(routineId, currentRoutineName);
        } catch (error) {
            console.error('Error removing exercise from routine:', error);
            if (routineExerciseMessage) displayMessage('routine-exercise-message', error.message || 'Error removing exercise from routine.', true);
        }
    }

    // Initial loads
    if (routineList) loadRoutines(); // Load routines on page load
    if (exerciseToSelect) loadExercisesForDropdown(); // Load exercises for the dropdown in the "add exercise to routine" form
    
    // Could add a cancel button to the main routine form that calls resetRoutineForm()
});
