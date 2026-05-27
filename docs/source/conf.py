# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'CoHAN-Nav2'
copyright = '2026, CoHAN-Nav2'
author = 'Phani Teja Singamaneni'

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.intersphinx',
]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

html_static_path = ['_static']
def setup(app):
    app.add_css_file("custom.css")

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'
html_show_sourcelink = False

html_theme_options = {
    "repository_url": "https://github.com/sphanit/CoHAN-Nav2",
    "use_repository_button": True,
    "use_edit_page_button": True,
}

html_context = {
    "display_github": True,
    "github_user": "sphanit",
    "github_repo": "CoHAN-Nav2",
    "github_version": "main",
    "conf_py_path": "/docs/source/",
}

# -- Options for EPUB output
epub_show_urls = 'footnote'

import os
import sys
# from exhale.utils import makeCustomSpecificationsMapping
sys.path.insert(0, os.path.abspath('.'))


extensions = ['breathe']
## USe exhale to rebuild api
# extensions = ['breathe', 'exhale']

# Path to Doxygen XML
breathe_projects = {
    "CoHAN-Nav2": "../docs/xml"
}
breathe_default_project = "CoHAN-Nav2"