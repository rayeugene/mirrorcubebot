from environment.sdf_generator import SDFGenerator
import pytest

def test_generate_link():
    test_1_target = '<link name="center"/>'
    test_1_value = SDFGenerator.generate_link(SDFGenerator.Attributes({"name":"center"}))
    print(SDFGenerator.Attributes({"name":"center"}).get_tag_string())
    assert test_1_target == test_1_value

    test_2_target = '<link name="box_0_0_0">\n\t<pose>-0.0125 -0.0125 -0.0125 0 0 0</pose>\n</link>'
    print(test_2_target)
    test_2_value = SDFGenerator.generate_link(SDFGenerator.Attributes({"name":"box_0_0_0"}), content="<pose>-0.0125 -0.0125 -0.0125 0 0 0</pose>")
    assert test_2_target == test_2_value