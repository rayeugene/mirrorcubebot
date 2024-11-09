import numpy as np
import matplotlib.colors as mcolors

class SDFGenerator():
    class Attributes():
        def __init__(self, attributes: dict):
            """
                args: attributes is a dict that maps from the attribute's string literal to its string value. don't use apostraphes!
            """
            self.attributes = attributes            
        
        def get_tag_string(self):
            return " " + " ".join(f'{attribute}="{value}"' for attribute, value in self.attributes.items()) if len(self.attributes.keys()) > 0 else ""

    def generate_box(pose, ):

        pass

    def generate_link(attributes:Attributes, content=""):
        return "<link" + attributes.get_tag_string() + "/>" if content=="" else \
            "<link" + attributes.get_tag_string() + ">\n" + SDFGenerator.tab_all_lines(content) + "\n</link>"
    
    def tab_all_lines(text: str, tab='\t'):
        return '\n'.join(f"{tab}{line}" for line in text.splitlines())
        