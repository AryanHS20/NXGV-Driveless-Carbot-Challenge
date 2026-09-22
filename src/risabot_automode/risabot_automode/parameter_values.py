"""Strict conversion using the parameter's declared ROS type."""
import json
import math


def coerce_parameter_value(text, type_id):
    if type_id == 4:
        return str(text)
    if type_id == 1:
        if str(text).lower() not in ('true', 'false'):
            raise ValueError('Expected true or false')
        return str(text).lower() == 'true'
    if type_id in (2, 3):
        number = float(text)
        if not math.isfinite(number):
            raise ValueError('Expected a finite number')
        if type_id == 2:
            if not number.is_integer():
                raise ValueError('Expected an integer')
            return int(number)
        return number
    if type_id in (5, 6, 7, 8, 9):
        items = json.loads(text)
        if not isinstance(items, list):
            raise ValueError('Expected a JSON array')
        scalar_type = {5:2, 6:1, 7:2, 8:3, 9:4}[type_id]
        result = [coerce_parameter_value(str(x), scalar_type) for x in items]
        if type_id == 5 and any(not 0 <= x <= 255 for x in result):
            raise ValueError('Byte values must be 0..255')
        return result
    raise ValueError('Parameter is not declared')
