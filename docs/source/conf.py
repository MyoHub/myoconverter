# Authors  :: Aleksi Ikkala (aleksi.ikkala@gmail.com), Huawei Wang (h.wang-2@utwente.nl)
#
# This source code is licensed under the Apache 2 license found in the
# LICENSE file in the root directory of this source tree.

import os
import sys

sys.path.insert(0, os.path.abspath("../.."))
#sys.path.insert(0, os.path.abspath("../../myoconverter"))


# -- Project information

project = 'MyoConverter'
copyright = "Aleksi Ikkala, Huawei Wang"
author = "Aleksi Ikkala, Huawei Wang"

release = '0.1'
version = '0.1.0'

# -- General configuration

extensions = [
    'sphinx.ext.duration',
    'sphinx.ext.doctest',
    'sphinx.ext.intersphinx',
    'sphinx.ext.viewcode'
    'autoapi.extension'
]
autoapi_type = 'python'
autoapi_dirs = ["../../myoconverter"]

intersphinx_mapping = {
    'python': ('https://docs.python.org/3/', None),
    'sphinx': ('https://www.sphinx-doc.org/en/master/', None),
}
intersphinx_disabled_domains = ['std']

templates_path = ['_templates']

# -- Options for HTML output

html_theme = 'sphinx_rtd_theme'
# html_static_path = ['_static']
html_logo = "images/logo-color-fit_horizontal.png"
html_theme_options = {
    'logo_only': True,
    'display_version': False,
}
# -- Options for EPUB output
epub_show_urls = 'footnote'
