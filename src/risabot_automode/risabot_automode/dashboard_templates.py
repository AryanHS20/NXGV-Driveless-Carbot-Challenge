# dashboard_templates.py
# Backward-compatible template constants, assembled from dashboard_panels/.
# Edit the panel fragments (HTML/JS) instead of this file.

from .dashboard_panels.registry import build_dashboard_html, build_teach_html

DASHBOARD_HTML = build_dashboard_html()
TEACH_HTML = build_teach_html()
