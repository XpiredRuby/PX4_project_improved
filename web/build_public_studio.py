"""Build the portable public studio from the same verified planner and editor."""
from pathlib import Path
import ast
import hashlib
import json
import re
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.aircraft import catalog  # noqa: E402


def build(destination):
    dest = Path(destination).resolve()
    dest.mkdir(parents=True, exist_ok=True)
    # Follow local Python imports from the preview; never bundle flight runners.
    names = {'__init__', 'preview'}
    todo = list(names)
    while todo:
        name = todo.pop()
        for node in ast.walk(ast.parse((ROOT / 'mission_planner' / f'{name}.py').read_text())):
            if isinstance(node, ast.ImportFrom) and node.level == 1 and node.module:
                if node.module not in names:
                    names.add(node.module)
                    todo.append(node.module)
    sources = [ROOT / 'mission_planner' / f'{name}.py' for name in sorted(names)]
    sources += sorted((ROOT / 'config/planner').rglob('*.json'))
    manifest = {'schema_version': 1, 'runtime': 'pyodide-0.27.7', 'files': []}
    for source in sources:
        relative = source.relative_to(ROOT).as_posix()
        target = dest / 'python' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(source.read_bytes())
        manifest['files'].append({'path': relative, 'sha256': hashlib.sha256(source.read_bytes()).hexdigest()})
    (dest / 'python-manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    (dest / 'aircraft-catalog.json').write_text(json.dumps(catalog(), allow_nan=False)+'\n')
    for name in ('planner-worker.js', 'browser-api.js', 'workspace.js', 'workspace.css', 'comparison.js', 'lifecycle-ui.js', 'simple-ui.js', 'motion.js'):
        shutil.copyfile(ROOT / 'web' / name, dest / name)
    shutil.copyfile(ROOT / 'mission_planner/draft.js', dest / 'draft.js')
    page = (ROOT / 'mission_planner/editor.html').read_text()
    replacements = {
        '<script src="/draft.js"></script>': '<script src="/draft.js"></script>\n<script src="/browser-api.js"></script>',
        'Draw a path. Review the smooth plan. Watch it in Gazebo.': 'Draw a path. Check the smooth plan. Open on your phone, tablet or computer.',
        '<div class="badge">Simulation only</div>': '<div class="badge">Browser planning</div>',
        'Nothing starts flying when you plan. You review the result before starting the simulation.': 'Planning runs on your device. Your drawing is not sent to a laptop. The first plan downloads the engine; keep this page open.',
        '3 · Try it in the simulator': '3 · Cloud simulation',
        'What happens: take off → follow your route → return to START → land and stop.': 'This page checks your route. A future connected Gazebo run will add takeoff, return to START and landing.',
        'Top view only. The drone takes off to your selected height before following this line.': 'Top view only. The planned route uses your selected height.',
        'Start simulation</button>': 'Cloud simulator not connected</button>',
        'Review the blue route below, then start the simulation.': 'Review the blue route and download your plan below.',
        'Gazebo is the virtual flight environment. Check the blue route above, then start the simulation. Afterwards, compare the purple flight path with your plan.': 'Gazebo is a separate virtual flight environment. Drawing, verification and downloads work here now. Running Gazebo from any device needs a connected cloud simulator.',
        'Checking local simulation connection…': 'Checking cloud simulation connection…',
        'Drawing and planning are available. Gazebo is not connected on this server, so simulation cannot start here.': 'Planning works on this device. Cloud Gazebo is not connected yet. No simulation has started.',
        'Gazebo has not run. Review and confirm a verified plan to start its simulation. The browser controls simulation only.': 'Cloud simulator not connected. You can plan and download a route; Gazebo has not run.',
        'Ready when you are. Review the blue route, tick the box below, then select Start simulation. Gazebo has not run.': 'Your plan passed its mathematical checks. Cloud Gazebo is not connected yet; this is not a simulated flight.',
        'After the route, the virtual drone returns to START and lands. Takeoff, return and landing add extra time.': 'This duration covers the route only. Takeoff, return and landing are not simulated here.',
        'Virtual flight only · The planner checks a mathematical model. Simulation results show how this virtual drone followed the plan.': 'Planning only · Mathematical checks use the declared model. Gazebo has not run. No real aircraft is connected.',
    }
    for before, after in replacements.items():
        if before not in page:
            raise ValueError('Editor changed; public copy needs review: '+before[:60])
        page = page.replace(before, after)
    page = page.replace('<label><input id="confirmPlan"', '<label hidden><input id="confirmPlan"')
    page = page.replace('<button id="cancelRehearsal" disabled>', '<button id="cancelRehearsal" hidden disabled>')
    page = page.replace('<p class="muted">Stop asks the virtual drone to land. Keep this page open while it lands and turns its motors off. This can take a little while.</p>', '')
    page = page.replace("$('lifecycle').textContent='What happens: take off → follow '+(closed?'the closed loop':'your route')+' → return to START → land and stop. The faint dotted return is indicative; the controller generates it during the run.';", "$('lifecycle').textContent='This page checks your route. A future connected Gazebo run will add takeoff, return to START and landing.';")
    page = re.sub(r'<style>.*?</style>', '<link rel="stylesheet" href="/workspace.css">', page, flags=re.S)
    page = page.replace('<script src="/browser-api.js"></script>', '<script src="/browser-api.js"></script>\n<script src="/workspace.js"></script>\n<script src="/comparison.js"></script>\n<script src="/lifecycle-ui.js"></script>\n<script src="/simple-ui.js"></script>')
    page = page.replace('<script src="/simple-ui.js"></script>', '<script src="/simple-ui.js"></script>\n<script src="/motion.js"></script>')
    (dest / 'index.html').write_text(page)
    print(f'Built {len(sources)} portable planner files at {dest}')


if __name__ == '__main__':
    build(sys.argv[1])
