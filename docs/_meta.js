// Section navigation for the RAMMP docs site.
//
// The hub copies this file in alongside the pages; its own copy is generated and
// gitignored, so THIS is the source of truth for ordering and titles.
//
// Ordered for a newcomer: start, then reference, then the deeper guides, with
// troubleshooting last so it is findable when something breaks.
//
// Only user-facing pages belong here. Operational records -- the on-robot runbook
// and the dated handoff notes -- stay in the repo and are excluded in the hub's
// sources.yml, along with docs/superpowers/ (specs and plans).
export default {
  index: 'Introduction',
  'getting-started': 'Getting started',
  interface: 'Interface reference',
  'guide-goto-actions': 'Planned moves',
  'guide-goto-ee-pose': 'Going to an EE pose',
  troubleshooting: 'Troubleshooting'
}
