"""
GeoShield -- production-path tests for tunnel / bridge / lanes propagation.

BUG-04 follow-up: the differential comparator could already see these fields,
but `lanelet2_adapter.to_segments()` hard-coded tunnel=None, bridge=None and
lanes=1, so edits to the source relation tags never reached `compare()`.

These tests build real Lanelet2 .osm XML, load it through the same
`load_map` alias the differential CLI uses (which is `lanelet2_adapter.load`),
and then run the real `compare()`. No RoadSegment is constructed by hand.
"""

import pytest

from differential_verify import compare, load_map


# One lanelet: left boundary way 11 (nodes 1-2), right boundary way 12
# (nodes 3-4). local_x/local_y make the width a metric 3.0 m.
_NODES = """  <node id="1" lat="35.6900" lon="139.6900">
    <tag k="local_x" v="0"/><tag k="local_y" v="0"/>
  </node>
  <node id="2" lat="35.6901" lon="139.6900">
    <tag k="local_x" v="0"/><tag k="local_y" v="10"/>
  </node>
  <node id="3" lat="35.6900" lon="139.6903">
    <tag k="local_x" v="3"/><tag k="local_y" v="0"/>
  </node>
  <node id="4" lat="35.6901" lon="139.6903">
    <tag k="local_x" v="3"/><tag k="local_y" v="10"/>
  </node>"""

_WAYS = """  <way id="11"><nd ref="1"/><nd ref="2"/></way>
  <way id="12"><nd ref="3"/><nd ref="4"/></way>"""

# Attributes that are identical on both sides of every scenario below. They
# give the loader realistic content and prove unchanged attributes stay quiet.
_BASE_TAGS = {"subtype": "road", "speed_limit": "50", "one_way": "yes"}


def _osm(overrides=None, drop=()):
    """A one-lanelet Lanelet2 map, with context tags added/removed as asked."""
    tags = dict(_BASE_TAGS)
    tags.update(overrides or {})
    for key in drop:
        tags.pop(key, None)
    tag_lines = "\n".join(f'    <tag k="{k}" v="{v}"/>'
                          for k, v in tags.items())
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<osm version="0.6">\n'
        f'{_NODES}\n{_WAYS}\n'
        '  <relation id="100">\n'
        '    <tag k="type" v="lanelet"/>\n'
        f'{tag_lines}\n'
        '    <member type="way" ref="11" role="left"/>\n'
        '    <member type="way" ref="12" role="right"/>\n'
        '  </relation>\n'
        '</osm>\n'
    )


def _load(tmp_path, name, overrides=None, drop=()):
    path = tmp_path / name
    path.write_text(_osm(overrides, drop), encoding="utf-8")
    return load_map(str(path))


def _diff(tmp_path, prev_overrides=None, cand_overrides=None,
          prev_drop=(), cand_drop=()):
    prev = _load(tmp_path, "prev.osm", prev_overrides, prev_drop)
    cand = _load(tmp_path, "cand.osm", cand_overrides, cand_drop)
    return compare(prev, cand)


class TestAdapterAttributeNormalisation:
    """The adapter must preserve absence as unknown, not invent a default."""

    def test_absent_context_tags_stay_unknown(self, tmp_path):
        (seg,) = _load(tmp_path, "clean.osm")
        assert seg.tunnel is None
        assert seg.bridge is None
        # NOT 1: a lanelet is one lane, but the `lanes` tag is a separate claim.
        assert seg.lanes is None

    def test_explicit_values_are_parsed(self, tmp_path):
        (seg,) = _load(tmp_path, "tagged.osm",
                       {"tunnel": "yes", "bridge": "no", "lanes": "3"})
        assert seg.tunnel is True
        assert seg.bridge is False
        assert seg.lanes == 3

    def test_absent_and_explicit_false_are_distinct(self, tmp_path):
        (absent,) = _load(tmp_path, "absent.osm")
        (explicit_false,) = _load(tmp_path, "false.osm",
                                  {"tunnel": "no", "bridge": "no"})

        assert absent.tunnel is None
        assert absent.bridge is None
        assert explicit_false.tunnel is False
        assert explicit_false.bridge is False
        # Raw tags retain the difference even if a normalised value is None.
        assert "tunnel" not in absent.raw_tags
        assert explicit_false.raw_tags["tunnel"] == "no"


class TestProductionPathContextAttributes:
    """Real adapter ingestion plus real differential comparison."""

    def test_clean_maps_accept(self, tmp_path):
        rep = _diff(tmp_path)
        assert rep.changes == []
        assert rep.added == []
        assert rep.removed == []
        assert rep.decide() == "ACCEPT"

    def test_unchanged_context_attributes_accept(self, tmp_path):
        same = {"tunnel": "yes", "bridge": "no", "lanes": "2"}
        rep = _diff(tmp_path, same, same)
        assert rep.changes == []
        assert rep.decide() == "ACCEPT"

    @pytest.mark.parametrize("prev,cand,field", [
        ({}, {"tunnel": "yes"}, "tunnel"),          # introduced
        ({"tunnel": "yes"}, {}, "tunnel"),          # removed
        ({"tunnel": "yes"}, {"tunnel": "no"}, "tunnel"),  # changed
        ({}, {"bridge": "yes"}, "bridge"),
        ({"bridge": "yes"}, {}, "bridge"),
        ({"bridge": "yes"}, {"bridge": "no"}, "bridge"),
        ({}, {"lanes": "3"}, "lanes"),
        ({"lanes": "3"}, {}, "lanes"),
        ({"lanes": "3"}, {"lanes": "2"}, "lanes"),
    ])
    def test_context_attribute_edit_is_never_accepted(self, tmp_path, prev, cand, field):
        rep = _diff(tmp_path, prev, cand)

        assert rep.decide() != "ACCEPT"
        assert rep.decide() == "REVIEW"
        assert len(rep.changes) == 1
        ch = rep.changes[0]
        assert ch.field_name == field
        assert ch.verdict == "SUSPECT"
        assert ch.reason

    def test_introduced_tunnel_uses_adapter_value(self, tmp_path):
        rep = _diff(tmp_path, {}, {"tunnel": "yes"})
        ch = rep.changes[0]
        assert ch.field_name == "tunnel"
        assert ch.previous is None
        assert ch.candidate is True
        assert "INTRODUCED" in ch.reason

    def test_removed_tunnel_is_a_genuine_removal(self, tmp_path):
        rep = _diff(tmp_path, {"tunnel": "yes"}, {})
        ch = rep.changes[0]
        assert ch.field_name == "tunnel"
        assert ch.previous is True
        assert ch.candidate is None
        assert "REMOVED" in ch.reason
        assert "parser limitation" not in ch.reason

    def test_explicit_false_introduction_is_review(self, tmp_path):
        """Absent -> explicit 'no' is still a source change, not equality."""
        rep = _diff(tmp_path, {}, {"tunnel": "no"})
        ch = rep.changes[0]
        assert ch.previous is None
        assert ch.candidate is False
        assert rep.decide() == "REVIEW"

    def test_unparsable_lanes_tag_is_a_parser_limitation(self, tmp_path):
        rep = _diff(tmp_path, {"lanes": "2"}, {"lanes": "not-a-number"})
        ch = rep.changes[0]
        assert ch.field_name == "lanes"
        assert ch.verdict == "SUSPECT"
        assert "parser limitation" in ch.reason
        assert rep.decide() == "REVIEW"

    def test_unparsable_tunnel_value_is_a_parser_limitation(self, tmp_path):
        rep = _diff(tmp_path, {"tunnel": "yes"}, {"tunnel": "unrecognised"})
        ch = rep.changes[0]
        assert ch.field_name == "tunnel"
        assert ch.verdict == "SUSPECT"
        assert "parser limitation" in ch.reason
        assert rep.decide() == "REVIEW"
