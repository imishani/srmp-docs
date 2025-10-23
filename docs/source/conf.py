# Configuration file for the Sphinx documentation builder.

# -- Project information

project = 'SRMP'
copyright = '2021, Graziella'
author = 'Itamar Mishani, Yorai Shaoul, Ramkumar Natarajan'

release = '0.0.5'
version = '0.0.5'

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

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'

# Copy static files from assets directory
html_static_path = ['_static']

# Copy additional files (including videos and gifs)
html_extra_path = ['_static/assets/srmp_moveit2.mp4', '_static/assets/srmp_moveit2.gif']

# -- Options for EPUB output
epub_show_urls = 'footnote'
