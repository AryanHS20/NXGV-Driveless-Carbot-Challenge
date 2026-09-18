"""Pin the assembled dashboard against checked-in goldens (no ROS).

Any intentional visual change must regenerate tests/golden_dashboard.html
and tests/golden_teach.html deliberately: render via
risabot_automode.dashboard_panels.registry and overwrite the goldens.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'risabot_automode'))

from risabot_automode.dashboard_panels import registry
from risabot_automode import dashboard_templates


def read_golden(name):
    path = os.path.join(os.path.dirname(__file__), name)
    with open(path, encoding='utf-8', newline='') as handle:
        return handle.read()


class PanelAssemblyTests(unittest.TestCase):
    def test_dashboard_byte_identical(self):
        self.assertEqual(registry.build_dashboard_html(),
                         read_golden('golden_dashboard.html'))

    def test_teach_byte_identical(self):
        self.assertEqual(registry.build_teach_html(),
                         read_golden('golden_teach.html'))

    def test_shim_constants_match_builder(self):
        self.assertEqual(dashboard_templates.DASHBOARD_HTML,
                         registry.build_dashboard_html())
        self.assertEqual(dashboard_templates.TEACH_HTML,
                         registry.build_teach_html())

    def test_registry_well_formed(self):
        self.assertEqual(len(registry.ORDER), 29)
        self.assertEqual(len(set(registry.ORDER)), 29)
        names = registry.plugin_names()
        self.assertEqual(len(names), 29)
        self.assertIn('v4views', names)
        self.assertIn('js_sim', names)
        for rel_path in registry.ORDER + [registry.TEACH_PAGE]:
            full = os.path.join(os.path.dirname(registry.__file__),
                                *rel_path.split('/'))
            self.assertTrue(os.path.isfile(full), rel_path)
            self.assertGreater(os.path.getsize(full), 0, rel_path)


if __name__ == '__main__':
    unittest.main()
