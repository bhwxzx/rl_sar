"""Small lexical checks for wiring that offline behavior tests cannot exercise.

These are not C++/CMake parsers and do not prove control flow. Keep behavior
tests as the authority for the invoked components.
"""

import re
import shlex


CPP_TOKEN = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|\w+|::|->|<=|>=|==|!=|\S')
CPP_COMMENT = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*.*?\*/', re.S)


def cpp_index(source: str, snippet: str) -> int:
    # Preserve offsets and quoted literals, but never match commented-out code.
    code = CPP_COMMENT.sub(
        lambda match: " " * len(match[0]) if match[0].startswith(("//", "/*")) else match[0],
        source,
    )
    pattern = r"\s*".join(re.escape(token) for token in CPP_TOKEN.findall(snippet))
    match = re.search(pattern, code)
    if match is None:
        raise AssertionError(f"Missing C++ wiring: {snippet}")
    return match.start()


def cpp_region(source: str, begin: str, end: str) -> str:
    return source[cpp_index(source, begin):cpp_index(source, end)]


def require_cpp_order(source: str, *snippets: str) -> None:
    positions = [cpp_index(source, snippet) for snippet in snippets]
    if positions != sorted(set(positions)):
        raise AssertionError("C++ wiring order differs: " + " -> ".join(snippets))


def cmake_calls(source: str) -> list[tuple[str, list[str], int]]:
    """Read flat declarations used by dependency-discovery checks, not a CMake AST."""
    return [
        (match[1].lower(), shlex.split(match[2], comments=True), match.start())
        for match in re.finditer(r"(?m)^\s*(\w+)\s*\(([^()]*)\)", source)
    ]
