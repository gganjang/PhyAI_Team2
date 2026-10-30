"""Task names derived from measured block centers and a configured target."""
import math


def zone_position(value):
    if value is None:
        return None
    if (not isinstance(value, (list, tuple)) or len(value) != 2
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in value)):
        raise ValueError('stacking_zone_position requires finite X/Y in map_frame.')
    return list(value)


def name_inventory(workspace, search, zone):
    """Assign final public IDs without changing internal association identities."""
    finished = search['state'] == 'search_finished'
    workspace['stacking_order'] = {
        'assigned': finished and zone is not None, 'frame': workspace['frame'],
        'zone_position': list(zone) if zone is not None else None, 'metric': 'xy_center_distance', 'ids': []}
    if zone is None or not finished:
        return
    tracks = workspace['tracks']
    # Coordinates break exact distance ties independently of detection order.
    tracks.sort(key=lambda t: (math.dist(t['position'][:2], zone),
                               *t['position'][:2], t['id']))
    names = {}
    for index, track in enumerate(tracks):
        name = 'block_basement' if index == 0 else f'block_{index}'
        names[track['id']] = name
        track.update(track_id=track['id'], id=name, name=name, stack_index=index,
                     distance_to_stacking_zone=math.dist(track['position'][:2], zone))
    workspace['stacking_order']['ids'] = [track['id'] for track in tracks]
    assessment = search.get('assessment')
    if assessment:
        for block in assessment['blocks']:
            block['track_id'] = block['id']
            block['id'] = names.get(block['id'], block['id'])
        for index, reason in enumerate(assessment['reasons']):
            prefix, separator, detail = reason.partition(':')
            if separator and prefix in names:
                assessment['reasons'][index] = names[prefix]+':'+detail
