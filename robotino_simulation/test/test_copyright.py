# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import pytest
from ament_copyright.main import main


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason="No copyright header has been placed in the generated source file.")
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found errors"
