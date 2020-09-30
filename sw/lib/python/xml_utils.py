from lxml import etree


def get_attrib(node: etree.Element, tag: str, typ=None):
    """XML Element attribute getter which is insensitive to case, handle default value and type conversion"""
    try:
        value = node.attrib[tag.lower()]
    except KeyError:
        value = node.attrib[tag.upper()]

    if typ is None:
        return value
    else:
        return typ(value)


def get_attrib_default(node: etree.Element, tag: str, default, typ=None):
    try:
        value = get_attrib(node, tag, typ=typ)
    except KeyError:
        value = default

    if typ is not None and default is not None:
        return typ(value)
    else:
        return value
