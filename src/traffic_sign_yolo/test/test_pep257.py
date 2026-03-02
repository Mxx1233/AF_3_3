import sys

from ament_pep257.main import main

import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    argv = ['.']
    rc = main(argv=argv)

    if rc != 0:
        print('PEP257 Error')
        sys.stdout.flush()
    assert rc == 0, 'Found code style errors / warnings'
