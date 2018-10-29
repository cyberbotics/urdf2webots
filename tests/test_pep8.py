"""Test module of the PEP8 format in the tests."""
import unittest

import glob
import pep8
import os

packageDirectory = os.path.abspath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))


class CustomReport(pep8.StandardReport):
    """Collect report, and overload the string operator."""

    results = []

    def get_file_results(self):
        """Overload this function to collect the report."""
        if self._deferred_print:
            self._deferred_print.sort()
            for line_number, offset, code, text, _ in self._deferred_print:
                self.results.append({
                    'path': self.filename,
                    'row': self.line_offset + line_number,
                    'col': offset + 1,
                    'code': code,
                    'text': text,
                })
        return self.file_errors

    def __str__(self):
        """Overload the string operator."""
        output = 'Report:\n'
        for result in self.results:
            output += '%s:%d:%d: %s: %s\n' % (
                result['path'],
                result['row'],
                result['col'],
                result['code'],
                result['text']
            )
        return output


class TestCodeFormat(unittest.TestCase):
    """Unit test of the PEP8 format in the tests."""

    def test_pep8_conformance(self):
        """Test that the tests are PEP8 compliant."""
        checker = pep8.StyleGuide(
            quiet=True,
            paths=glob.glob(packageDirectory + os.sep + '*.py') + glob.glob(packageDirectory + os.sep + 'tests' + os.sep + '*.py'),
            reporter=CustomReport
        )
        checker.options.ignore = ('E501')  # E501: line too long (> 80 characters)
        report = checker.check_files()
        self.assertEqual(
            report.total_errors, 0,
            msg='PEP8 errors:\n%s' % (report)
        )
