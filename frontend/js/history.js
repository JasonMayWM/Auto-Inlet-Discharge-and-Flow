document.addEventListener('DOMContentLoaded', function() {
    const historyList = document.getElementById('history-list');
    const historyMessage = document.getElementById('history-message');

    // Fetch and display workout history
    async function loadHistory() {
        const historyTableBody = document.getElementById('history-list'); // This is now a tbody
        if (!historyTableBody) return;
        
        try {
            const historyItems = await fetchData('/history');
            historyTableBody.innerHTML = ''; // Clear loading message or previous rows
            
            if (historyItems.length === 0) {
                historyTableBody.innerHTML = '<tr><td colspan="8">No workout history found. Go log a workout!</td></tr>';
                return;
            }
            
            historyItems.forEach(item => {
                const row = historyTableBody.insertRow();
                
                const type = item.routine_id ? 'Routine' : (item.exercise_id ? 'Exercise' : 'N/A');
                const name = item.routine_name || item.exercise_name || 'N/A';
                
                // Handle performed_reps for AMRAP display
                let repsDisplay = item.performed_reps;
                if (item.performed_reps === null && item.performed_sets !== null) { // Check sets to distinguish from no data
                    repsDisplay = 'AMRAP';
                } else if (item.performed_reps === null) {
                    repsDisplay = 'N/A';
                }

                row.innerHTML = `
                    <td>${new Date(item.performed_at).toLocaleString()}</td>
                    <td>${type}</td>
                    <td>${name}</td>
                    <td>${item.performed_sets !== null ? item.performed_sets : 'N/A'}</td>
                    <td>${repsDisplay}</td>
                    <td>${item.weight_lifted_kg !== null ? item.weight_lifted_kg.toFixed(2) + ' kg' : 'N/A'}</td>
                    <td>${item.duration_completed_minutes !== null ? item.duration_completed_minutes + ' min' : 'N/A'}</td>
                    <td>${item.notes || ''}</td>
                `;
            });
        } catch (error) {
            historyTableBody.innerHTML = '<tr><td colspan="8">Error loading workout history.</td></tr>';
            console.error('Error loading history:', error);
            if (historyMessage) displayMessage('history-message', 'Error loading workout history.', true);
        }
    }

    // Initial load of history
    loadHistory();
});
