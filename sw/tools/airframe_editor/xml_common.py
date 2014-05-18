from __future__ import print_function


def indent(elem, level=0, more_sibs=False):
    i = "\n"
    num_kids = len(elem)
    if level:
        i += (level-1) * '  '
    #print(level, elem.tag, num_kids, more_sibs)
    if num_kids:
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
            if level:
                elem.text += '  '
        count = 0
        for kid in elem:
            indent(kid, level+1, count < num_kids - 1)
            count += 1
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
            if more_sibs:
                elem.tail += '  '
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
            if more_sibs:
                elem.tail += '  '
