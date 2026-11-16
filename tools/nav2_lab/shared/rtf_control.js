'use strict';
/* Single Nav2 RTF control shared by Planner and Tracker, independent of
   which tab currently owns a Gazebo process. Never scales ROS simulated Hz. */
(function() {
  const api = '/api/sim-speed';
  let saved = null;
  let dirty = false;
  let saving = false;
  let current = null;

  const label = (number) =>
    typeof number === 'number' && Number.isFinite(number) ?
      number.toFixed(2) + '×' : '—';

  function init() {
    const container = document.querySelector('main .content');
    if (!container || document.getElementById('lab-rtf-control')) return;
    const section = document.createElement('section');
    section.id = 'lab-rtf-control';
    section.className = 'panel lab-rtf-panel';
    section.innerHTML =
      '<div class="between"><h2>Gazebo speed cap</h2>' +
      '<span>Shared · Planner + Tracker</span></div>' +
      '<div class="lab-rtf-row">' +
      '<label for="lab-rtf-slider" class="lab-rtf-label">Maximum real-time factor</label>' +
      '<input type="range" id="lab-rtf-slider" min="0.10" max="1.00" step="0.05" value="0.25" ' +
      'aria-label="Shared maximum Gazebo real-time factor" />' +
      '<strong id="lab-rtf-value">0.25×</strong>' +
      '<button type="button" class="btn secondary" id="lab-rtf-save">Save cap for both tabs</button></div>' +
      '<p id="lab-rtf-note" class="fineprint">Loading shared Nav2 speed cap…</p>' +
      '<p id="lab-rtf-applied" class="fineprint" aria-live="polite">' +
      'Checking which Gazebo sessions need a restart…</p>';
    container.insertBefore(section, container.firstChild);

    const slider = section.querySelector('#lab-rtf-slider');
    const value = section.querySelector('#lab-rtf-value');
    const button = section.querySelector('#lab-rtf-save');
    const note = section.querySelector('#lab-rtf-note');
    const appliedText = section.querySelector('#lab-rtf-applied');

    function show() {
      const requested = Number(slider.value);
      value.textContent = label(requested);
      const pending = dirty && saved !== null && Math.abs(saved - requested) > 1e-6;
      button.disabled = saving || !pending || saved === null;
      const equivalent = 'At ' + label(requested) +
        ' cap, 30 sim-Hz corresponds to at most ' + (30 * requested).toFixed(2) +
        ' wall-Hz, and 100 sim-Hz to at most ' + (100 * requested).toFixed(2) +
        ' wall-Hz. Actual rates follow measured RTF, not the requested cap.';
      note.textContent = (pending ? 'Unsaved slider setting. ' : '') +
        equivalent + ' ROS rates and production calibration are not rescaled.';

      if (current) {
        const states = ['planner', 'tracker'].map((tab) => {
          const applied = current.applied?.[tab];
          const title = tab === 'planner' ? 'Planner' : 'Tracker';
          if (typeof applied !== 'number') return title + ': stopped';
          const needsRestart =
            Math.abs(applied - current.max_rtf) > 1e-6;
          return title + ': running with ' + label(applied) +
            ' cap' + (needsRestart ? ' (restart required)' : '');
        });
        appliedText.textContent = states.join(' · ') +
          '. Changing a cap does not restart either simulation.';
        appliedText.classList.toggle('lab-rtf-warning',
          states.some(text => text.includes('restart required')));
      }
    }

    async function refresh() {
      if (saving) return;
      try {
        const response = await fetch(api, {cache: 'no-store'});
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || 'HTTP ' + response.status);
        current = result;
        saved = Number(result.max_rtf);
        if (!dirty) slider.value = String(saved);
        else if (Math.abs(Number(slider.value) - saved) <= 1e-6)
          dirty = false;
        show();
      } catch (error) {
        note.textContent = 'Unable to read shared Gazebo speed cap: ' + error.message;
      }
    }

    slider.addEventListener('input', () => {
      dirty = true;
      show();
    });
    button.addEventListener('click', async () => {
      if (saving) return;
      saving = true;
      show();
      try {
        const response = await fetch(api, {
          method: 'POST',
          headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({max_rtf: Number(slider.value)})
        });
        const result = await response.json();
        if (!response.ok) throw new Error(result.error || 'HTTP ' + response.status);
        current = result;
        saved = Number(result.max_rtf);
        dirty = false;
        slider.value = String(saved);
        show();
        note.textContent = 'Saved ' + label(saved) +
          ' for both Nav2 tabs. Any tab running with a different applied cap ' +
          'must restart its own Gazebo process. Sensor/controller simulated-Hz ' +
          'targets remain unchanged.';
      } catch (error) {
        note.textContent = 'Unable to save shared Gazebo speed cap: ' + error.message;
      } finally {
        saving = false;
        show();
      }
    });

    // Two browser tabs can be open at once. Refresh without overriding
    // an unsaved local slider edit.
    window.addEventListener('focus', refresh);
    setInterval(() => {
      if (document.visibilityState === 'visible') refresh();
    }, 8000);
    refresh();
  }
  if (document.readyState === 'loading')
    document.addEventListener('DOMContentLoaded', init);
  else init();
})();
