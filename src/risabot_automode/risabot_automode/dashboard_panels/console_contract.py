"""Explicit UI capabilities. Missing runtime adapters are not successful actions."""
import copy
import re

from .console_inventory import INVENTORY


GAPS = {
    'launch': 'Not connected: this checkout has no calibrate/race lifecycle supervisor. The launch/control task must provide stop, start and observed readiness acknowledgements. Use neither mockup mode as proof of the running stack.',
    'motion': 'Not connected: no acknowledged dashboard authority service is available. Use the physical controller/E-stop. This preview cannot stop or start motors.',
    'calibration': 'Not connected: the calibration step runner, measured results and dependency-aware session save/restore adapter are missing. No calibration PASS is inferred from visiting this page.',
    'lap': 'Not connected: recording needs a verified venue-frame pose source and acknowledgement that map feedback is off. Import a measured venue lap for draft editing; do not use simulated dashboard odometry.',
    'tuning': 'Session-aware parameter validation, save and rollback are not connected here. Existing live tuning remains on the legacy dashboard; this preview does not alter it.',
}


def short_label(label):
    # Remove example success badges/dates and explanatory small text, not controls.
    for long, short in (
        ('Restart camera drivers', 'Restart camera drivers'),
        ('Keep previous value', 'Keep previous value'),
        ('Show expected corners', 'Show expected corners'),
        ('Calibrate IMU hardware', 'Calibrate IMU hardware'),
        ('Swap left and right', 'Swap left and right'),
        ('Auto-fit', 'Auto-fit'), ('Refit', 'Refit'), ('Flip', 'Flip'),
    ):
        if label.startswith(long):
            return short
    return label.replace(' ✓', '')


def catalog():
    result = copy.deepcopy(INVENTORY)
    for screen in result['screens']:
        for index, button in enumerate(screen['buttons']):
            label = short_label(button['label'])
            button.update(id=f"{screen['id']}:{index}", mock_label=button['label'], label=label,
                          action=None, reason=GAPS['calibration'], disabled=True)
            if button.get('navigation'):
                button.update(action='navigate', reason=None, disabled=False)
            elif screen['id'] == 'cal-11':
                action = {'Auto-fit': 'fit', 'Refit': 'refit', 'Redo': 'refit',
                          'Discard lap': 'discard', 'Reset selected handle': 'reset_handle',
                          'Save': 'save_map', 'Save map': 'save_map',
                          'Full build (new venue)': 'full_build'}.get(label)
                if action:
                    button.update(action=action, reason=None, disabled=False)
                elif label in ('Record lap', 'Stop lap'):
                    button['reason'] = GAPS['lap']
                elif label.startswith('Quick'):
                    button['reason'] = 'Quick point-pair realignment is not implemented in the supplied script. Full lap fitting and manual handle editing are available.'
            elif screen['id'] == 'cal-12':
                if re.fullmatch(r'P[0-3]', label):
                    button.update(action='point', reason=None, disabled=False)
                else:
                    action = {'Review': 'review', '−5°': 'rotate', '−1°': 'rotate',
                              '+1°': 'rotate', '+5°': 'rotate', 'Flip': 'flip',
                              'Reset pose': 'reset_pose', 'Save mission': 'save_mission',
                              'Save': 'save_mission', 'Redo': 'plan'}.get(label)
                    if action:
                        button.update(action=action, reason=None, disabled=False)
            elif screen['id'] == 'drive':
                button['reason'] = GAPS['motion']
            elif screen['id'] == 'tuning':
                button['reason'] = GAPS['tuning']
            elif screen['id'] == 'cal-1' and label in ('Run check', 'Redo'):
                button.update(action='sensor_check', reason=None, disabled=False)
            elif screen['id'] in ('events', 'percplan'):
                button.update(action='filter', reason=None, disabled=False)
        for field in screen['fields']:
            field['available'] = (screen['id'] == 'cal-12' and field['type'] == 'checkbox')
    result['gaps'] = GAPS
    result['preview'] = True
    return result
