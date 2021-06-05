#!/usr/bin/env python
"""A module for stripping HTML tags from strings"""

from HTMLParser import HTMLParser
from StringIO import StringIO


# pylint: disable=super-init-not-called
# If the super is called, stuff breaks, this works. Not great, but ok for now
class MLStripper(HTMLParser):
    """A class to strip out tags

    taken from: https://stackoverflow.com/a/925630/5274985
    """

    def __init__(self):
        self.reset()
        self.text = StringIO()

    def handle_data(self, data):
        """ingest data"""
        self.text.write(data)

    def get_data(self):
        """return data"""
        return self.text.getvalue()


def strip_tags(html):
    """strip html tags"""
    stripper = MLStripper()
    stripper.feed(html)
    return stripper.get_data()
