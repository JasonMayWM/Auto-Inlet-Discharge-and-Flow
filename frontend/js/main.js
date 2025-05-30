// Basic JS for shared functionality (if any)
// For example, highlighting active navigation link

document.addEventListener('DOMContentLoaded', function() {
    const navLinks = document.querySelectorAll('nav ul li a');
    const currentPath = window.location.pathname.split('/').pop();

    navLinks.forEach(link => {
        if (link.getAttribute('href') === currentPath) {
            link.classList.add('active'); // Assuming 'active' class is defined in CSS for styling
        }
    });

    // Add 'active' class style to css/style.css if needed
    // nav a.active { font-weight: bold; text-decoration: underline; }

    // If we are on index.html, load the next workout
    if (document.getElementById('next-workout-info')) {
        loadNextWorkout();
    }
});

async function loadNextWorkout() {
    const infoEl = document.getElementById('next-workout-info');
    const exercisesEl = document.getElementById('next-workout-exercises');
    try {
        const data = await fetchData('/next-workout'); // fetchData is already defined below
        if (infoEl) infoEl.textContent = data.message || data.workout_name || "No specific workout name"; 
        
        if (data.exercises && data.exercises.length > 0) {
            if (exercisesEl) {
                exercisesEl.innerHTML = ''; // Clear previous
                data.exercises.forEach(ex => { // ex is now an object
                    const li = document.createElement('li');
                    let displayText = ex.original_scheduled_name || ex.name;
                        
                        // Add target sets and reps
                        if (ex.target_sets !== undefined && ex.target_sets !== null) {
                            displayText += `: ${ex.target_sets} sets of `;
                            if (ex.target_reps === null) {
                                displayText += `AMRAP`;
                            } else {
                                displayText += `${ex.target_reps} reps`;
                            }
                        } else {
                            displayText += `: (Sets/Reps N/A)`; // Fallback if no set/rep data
                        }

                        // Add suggested weight
                    if (ex.suggested_weight !== undefined && ex.suggested_weight !== null) {
                            displayText += ` at ${ex.suggested_weight}`;
                        if (!ex.name.toLowerCase().includes("chin-ups") && ex.suggested_weight !== "Bodyweight") {
                            displayText += ` kg`;
                        }
                        }

                        // Add weight calculation details for clarity
                        if (typeof ex.current_weight === 'number' && typeof ex.weight_increment === 'number' && ex.suggested_weight !== "Bodyweight" && ex.suggested_weight !== "N/A - Exercise not in DB" && ex.suggested_weight !== "N/A - Routine context error") {
                             if (ex.is_80_percent) {
                                displayText += ` (80% of ${ex.current_weight + ex.weight_increment} kg)`;
                             } else if (ex.weight_increment !== 0) {
                                displayText += ` (current: ${ex.current_weight.toFixed(1)} kg + ${ex.weight_increment.toFixed(1)} kg)`;
                             } else if (ex.current_weight > 0) { // e.g. holding static weight for weighted chins
                                displayText += ` (current: ${ex.current_weight.toFixed(1)} kg)`;
                             }
                    }
                    li.textContent = displayText;
                    exercisesEl.appendChild(li);
                });
            }
            // Store for log_workout.js
            sessionStorage.setItem('nextWorkoutDetails', JSON.stringify(data));
        } else {
            if (exercisesEl) exercisesEl.innerHTML = '<li>No specific exercises listed for this workout.</li>';
            sessionStorage.removeItem('nextWorkoutDetails'); // Clear if no exercises
        }
    } catch (error) {
        console.error('Error loading next workout:', error);
        if (infoEl) infoEl.textContent = 'Could not load next workout.';
        if (exercisesEl) exercisesEl.innerHTML = '';
        sessionStorage.removeItem('nextWorkoutDetails');
    }
}

// --- Configuration ---
// It's often good practice to have a central place for API base URLs
const API_BASE_URL = '/api'; // Assuming Flask backend is served from the same domain

// --- Helper Functions ---
async function fetchData(endpoint, options = {}) {
    const url = `${API_BASE_URL}${endpoint}`;
    try {
        const response = await fetch(url, options);
        if (!response.ok) {
            const errorData = await response.json().catch(() => ({ message: response.statusText }));
            throw new Error(`HTTP error! status: ${response.status}, message: ${errorData.message || 'Unknown error'}`);
        }
        if (response.status === 204) { // No Content
            return null;
        }
        return await response.json();
    } catch (error) {
        console.error(`Error fetching data from ${url}:`, error);
        throw error;
    }
}

function displayMessage(elementId, message, isError = false) {
    const element = document.getElementById(elementId);
    if (element) {
        element.textContent = message;
        element.className = isError ? 'error-message' : 'success-message'; // Ensure these classes are in style.css
        // Clear message after some time
        setTimeout(() => {
            element.textContent = '';
            element.className = '';
        }, 5000);
    }
}
