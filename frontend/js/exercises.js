document.addEventListener('DOMContentLoaded', function() {
    const addExerciseForm = document.getElementById('add-exercise-form');
    const exerciseList = document.getElementById('exercise-list');
    const exerciseMessage = document.getElementById('exercise-message');
    const formTitle = document.querySelector('#add-exercise-form-container h3');
    const submitButton = addExerciseForm.querySelector('button[type="submit"]');
    let currentEditExerciseId = null; // To store the ID of the exercise being edited


    // Using displayMessage from main.js

    // Fetch and display all exercises
    async function loadExercises() {
        try {
            const exercises = await fetchData('/exercises');
            const exerciseListBody = document.getElementById('exercise-list'); // This is now a tbody
            exerciseListBody.innerHTML = ''; // Clear loading message or previous rows
            if (exercises.length === 0) {
                exerciseListBody.innerHTML = '<tr><td colspan="7">No exercises found. Add some!</td></tr>';
                return;
            }
            exercises.forEach(exercise => {
                const row = exerciseListBody.insertRow();
                row.innerHTML = `
                    <td>${exercise.name}</td>
                    <td>${exercise.description || ''}</td>
                    <td>${exercise.muscle_group || ''}</td>
                    <td>${exercise.equipment || ''}</td>
                    <td>${exercise.current_weight !== null ? exercise.current_weight.toFixed(2) : '0.00'}</td>
                    <td>${exercise.weight_increment !== null ? exercise.weight_increment.toFixed(2) : '0.00'}</td>
                    <td class="action-buttons">
                        <button class="edit" 
                                data-id="${exercise.id}" 
                                data-name="${exercise.name}" 
                                data-description="${exercise.description || ''}" 
                                data-muscle_group="${exercise.muscle_group || ''}" 
                                data-equipment="${exercise.equipment || ''}"
                                data-current_weight="${exercise.current_weight !== null ? exercise.current_weight : ''}"
                                data-weight_increment="${exercise.weight_increment !== null ? exercise.weight_increment : ''}">Edit</button>
                        <button class="delete" data-id="${exercise.id}">Delete</button>
                    </td>
                `;
                row.querySelector('.delete').addEventListener('click', () => deleteExercise(exercise.id));
                row.querySelector('.edit').addEventListener('click', (e) => populateFormForEdit(e.target.dataset));
            });
        } catch (error) {
            const exerciseListBody = document.getElementById('exercise-list');
            exerciseListBody.innerHTML = '<tr><td colspan="7">Error loading exercises.</td></tr>';
            console.error('Error loading exercises:', error);
            if (exerciseMessage) displayMessage('exercise-message', 'Error loading exercises.', true);
        }
    }

    // Handle Add New Exercise form submission
    if (addExerciseForm) {
        addExerciseForm.addEventListener('submit', async function(event) {
            event.preventDefault();
            const formData = new FormData(addExerciseForm);
            const rawData = Object.fromEntries(formData.entries());
            const data = {
                ...rawData,
                current_weight: parseFloat(rawData.current_weight) || 0.0,
                weight_increment: parseFloat(rawData.weight_increment) || 0.0
            };
            
            let method = 'POST';
            let url = '/exercises';
            let successMessage = `Exercise "${data.name}" added successfully!`;

            if (currentEditExerciseId) {
                method = 'PUT';
                url = `/exercises/${currentEditExerciseId}`;
                successMessage = `Exercise "${data.name}" updated successfully!`;
            }

            try {
                const result = await fetchData(url, {
                    method: method,
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(data)
                });
                if (exerciseMessage) displayMessage('exercise-message', successMessage);
                resetForm();
                loadExercises(); // Refresh the list
            } catch (error) {
                console.error(`Error ${currentEditExerciseId ? 'updating' : 'adding'} exercise:`, error);
                if (exerciseMessage) displayMessage('exercise-message', error.message || `Error ${currentEditExerciseId ? 'updating' : 'adding'} exercise.`, true);
            }
        });
    }
    
    function populateFormForEdit(exerciseData) {
        currentEditExerciseId = exerciseData.id;
        addExerciseForm.elements['name'].value = exerciseData.name;
        addExerciseForm.elements['description'].value = exerciseData.description;
        addExerciseForm.elements['muscle_group'].value = exerciseData.muscle_group;
        addExerciseForm.elements['equipment'].value = exerciseData.equipment;
        addExerciseForm.elements['current_weight'].value = exerciseData.current_weight || '0.0';
        addExerciseForm.elements['weight_increment'].value = exerciseData.weight_increment || '2.5'; // Default to 2.5 if not set
        
        if(formTitle) formTitle.textContent = 'Edit Exercise';
        if(submitButton) submitButton.textContent = 'Update Exercise';
        
        // Scroll to form for better UX
        addExerciseForm.scrollIntoView({ behavior: 'smooth' });
    }

    function resetForm() {
        addExerciseForm.reset();
        currentEditExerciseId = null;
        if(formTitle) formTitle.textContent = 'Add New Exercise';
        if(submitButton) submitButton.textContent = 'Add Exercise';
    }

    // Delete an exercise
    async function deleteExercise(exerciseId) {
        if (!confirm('Are you sure you want to delete this exercise?')) {
            return;
        }
        try {
            await fetchData(`/exercises/${exerciseId}`, { method: 'DELETE' });
            if (exerciseMessage) displayMessage('exercise-message', 'Exercise deleted successfully!');
            loadExercises(); // Refresh the list
        } catch (error) {
            console.error('Error deleting exercise:', error);
            if (exerciseMessage) displayMessage('exercise-message', error.message || 'Error deleting exercise.', true);
        }
    }
    
    // TODO: Implement editExercise function
    // Initial load of exercises
    if (exerciseList) { 
        loadExercises();
    }
    // Add a cancel button functionality to the form if desired
    // e.g., an explicit "Cancel Edit" button that calls resetForm()
});
