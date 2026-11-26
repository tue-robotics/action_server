"""
The rules that pick-up, hand-over and place share while a task is set up: does HERO already hold the object a step
asks for, which object must be fetched otherwise, and has the task already driven to where the step must happen.

Pick-up, hand-over and place must decide "already holding it" by the same rule: when they disagreed, a task such as
"find a drink, then hand it over" made them ask each other for a fetch forever.
"""
from robot_smach_states.util.entity_descriptions import description_satisfies

REFERENCE = "reference"  # the object of "it": whatever the previous steps provide


def _plain(description):
    """Only the type and the category of an object description (dict), without designators; None when it has neither

    Example: {'designator': <...>, 'type': None, 'category': 'drink'} -> {'category': 'drink'}
    """
    if not description:
        return None
    plain = {key: description[key] for key in ("type", "category") if description.get(key)}
    return plain or None


def holds_wanted_object(context, wanted, knowledge):
    """
    Whether the previous steps leave HERO holding an object that fits what this step asks for.

    Examples, with a context of a pick-up of {'type': 'coke'} (an arm and that object):
    wanted {'type': 'coke'}, {'category': 'drink'} and {'type': 'reference'} give True; {'category': 'food'} and
    {'type': 'water'} give False. Without an arm in the context it is always False.

    :param context: the parsed context of the step (arm_designator and object, an EntityDescription or None)
    :param wanted: EntityDescription of the object this step asks for; None or type 'reference' means "it"
    :param knowledge: the 'common' knowledge of the environment
    """
    if context.arm_designator is None or context.object is None:
        return False
    if wanted is None or wanted.type == REFERENCE:
        return True
    return description_satisfies(context.object, wanted, knowledge)


def object_to_fetch(semantics, context):
    """
    The object description a pick-up must fetch for this step, or None when it is not known (HERO then asks).

    - An object of the step itself is fetched as it is described: {'category': 'drink'} -> {'category': 'drink'}.
    - "It" ({'type': 'reference'}) is the object of the previous steps, by its type and category:
      after "find a drink", "it" -> {'category': 'drink'}. Without a previous object -> None.

    :param semantics: the semantics dict of the step
    :param context: the context dict of the step
    """
    wanted = semantics.get("object")
    if not wanted:
        return None
    if wanted.get("type") == REFERENCE:
        return _plain(context.get("object"))
    if not (wanted.get("type") or wanted.get("category")):
        return None
    return wanted


def _place_key(location):
    nested = location.get("location")
    return (location.get("id"), location.get("type"), nested.get("id") if isinstance(nested, dict) else None)


def reached(context, target):
    """
    Whether the previous step drove to this target, so the step can happen without driving again.

    Navigate-to marks the location it leaves in the context with 'navigated-to'; the target matches it by id, type and
    the location of a person. Example: after navigate-to {'id': 'cabinet'}, reached(context, {'id': 'cabinet'}) is
    True and reached(context, {'id': 'dinner_table'}) is False. A target of type 'reference' is reached by any
    location in the context.

    :param context: the context dict of the step
    :param target: the target-location dict of the step
    """
    location = context.get("location")
    if not location:
        return False
    if target.get("type") == REFERENCE:
        return True
    return bool(location.get("navigated-to")) and _place_key(location) == _place_key(target)
