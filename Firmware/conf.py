# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
from docutils import nodes
from sphinx.application import Sphinx

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "odrive"
copyright = "2025, KUNYI CHEN"
author = "KUNYI CHEN"
release = "1.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.intersphinx",
    "sphinx_design",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store", "env", ".tox", "README.md"]

intersphinx_mapping = {
    "python": ("https://docs.python.org/3/", None),
    "sphinx": ("https://www.sphinx-doc.org/en/master", None),
    "myst": ("https://myst-parser.readthedocs.io/en/latest", None),
}


linkcheck_allowed_redirects = {
    # All HTTP redirections from the source URI to the canonical URI will be treated as "working".
    r"https://sphinx-doc\.org/.*": r"https://sphinx-doc\.org/en/master/.*"
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "furo"
html_title = "ODrive Code Structure"
html_theme_options = {
    "source_repository": "https://github.com/KunYi/study_odrive",
    "source_branch": "main",
    "source_directory": "Firmware",
}


def lsp_role(name, rawtext, text, lineno, inliner, options={}, content=[]):
    """Link to sections within the lsp specification."""

    anchor = text.replace("/", "_")
    ref = f"https://microsoft.github.io/language-server-protocol/specifications/lsp/3.17/specification/#{anchor}"

    node = nodes.reference(rawtext, text, refuri=ref, **options)
    return [node], []


def setup(app: Sphinx):
    app.add_role("lsp", lsp_role)
