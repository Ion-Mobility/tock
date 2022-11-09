# Licensed under the Apache License, Version 2.0 or the MIT License.
# SPDX-License-Identifier: Apache-2.0 OR MIT

all: presentation.pdf

presentation.pdf: slides.md
	pandoc -s --pdf-engine xelatex -t beamer $< -o $@
