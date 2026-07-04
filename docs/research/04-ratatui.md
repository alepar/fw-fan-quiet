# Architecture Patterns for Scalable, Maintainable Ratatui TUIs (0.29+/0.30)

## TL;DR
- **Separate your model (state) from your view (rendering) and drive everything through a single message/action loop.** For a complex app, adopt either The Elm Architecture (a central `Model` + `Message` enum + `update` + pure `view`) or the component architecture from ratatui's official `component` template — both work well; the component template scales better to many independent panels.
- **You do not need async/tokio to build a responsive, complex TUI.** The reference app "bottom" uses plain OS threads (`std::thread`) with `std::sync::mpsc` channels and a `BottomEvent` enum to keep data collection off the render thread. Use tokio only if your workload is genuinely I/O-concurrency-heavy (many simultaneous network requests); otherwise threads + channels are simpler and equally responsive. The ratatui FAQ agrees: *"If the answer is not much, maybe it is simpler to not use async and avoiding tokio."*
- **The canonical main loop is: draw → wait for next event (input, tick, or background-task result) → update state → repeat**, with input read on a separate thread/task so heavy work never blocks rendering, plus a panic/color_eyre hook so the terminal is always restored.

## Key Findings
1. Ratatui is an **immediate-mode** library: every frame you re-render the entire UI from current state via `terminal.draw(|frame| ...)`, and ratatui diffs against the previous buffer so only changed cells are written. Your persistent application state lives outside ratatui entirely; the library only touches your code at `terminal.draw`.
2. Ratatui officially documents **three** application patterns — The Elm Architecture (TEA), Component Architecture, and Flux — and ships a `component` async template. The maintainers themselves caution that the website's TEA write-up is "pedagogical," so treat these as adaptable idioms, not dogma.
3. **State/rendering separation** is expressed through the `Widget`/`StatefulWidget` traits. Since 0.26, you can `impl Widget for &MyWidget` (or `&mut`) to keep state in the app and render by reference. `StatefulWidget` (used by `List`/`Table` via `ListState`/`TableState`) is the idiomatic way to keep selection/scroll state in the app rather than in the widget.
4. **Concurrency choice is a real fork in the road.** Threads + channels (bottom, gitui) vs. tokio tasks + async channels (atuin, the ratatui async/component templates, yazi). Both decouple the render loop from work; the differences are ecosystem fit, complexity, and whether you have many concurrent I/O operations.
5. Real apps prove both models: **bottom** = OS threads + `std::sync::mpsc`; **gitui** = worker thread pool + `crossbeam-channel`; **yazi** = fully async tokio with a task scheduler; **atuin** = tokio. There is no single "correct" answer.
6. **Testing** is well supported: `TestBackend` renders to an in-memory buffer you assert against, and `insta` snapshot tests capture the rendered terminal as text.

## Details

### 1. Model/View separation and the immediate-mode paradigm

**The core tension.** Ratatui redraws everything each frame ("immediate mode"), but your application data must persist between frames. The resolution is simple: keep all state in your own structs (the *model*), and make rendering a function of that state (the *view*). The only ratatui entry point is `terminal.draw(|frame| view(&model, frame))`; everything else — input, state updates, background work — is yours to structure.

Because the view is called fresh each frame, the cleanest discipline is to make it **pure**: for a given model state it always produces the same UI and never mutates global state. (`StatefulWidget` is the pragmatic exception — see below.)

**The Elm Architecture (TEA).** The canonical structure from ratatui's docs has four pieces:

- `Model` — a struct holding all app state.
- `Message` — an enum of everything that can happen (`Increment`, `Decrement`, `Quit`, ...).
- `update(&mut model, msg) -> Option<Message>` — the only place state changes; it may return another message to chain transitions (a mini finite-state-machine).
- `view(&model, frame)` — pure rendering.

```rust
#[derive(Debug, Default)]
struct Model {
    counter: i32,
    running_state: RunningState,
}

#[derive(Debug, Default, PartialEq, Eq)]
enum RunningState { #[default] Running, Done }

#[derive(PartialEq)]
enum Message { Increment, Decrement, Reset, Quit }

fn update(model: &mut Model, msg: Message) -> Option<Message> {
    match msg {
        Message::Increment => {
            model.counter += 1;
            if model.counter > 50 { return Some(Message::Reset); }
        }
        Message::Decrement => {
            model.counter -= 1;
            if model.counter < -50 { return Some(Message::Reset); }
        }
        Message::Reset => model.counter = 0,
        Message::Quit => model.running_state = RunningState::Done,
    };
    None
}

fn view(model: &Model, frame: &mut Frame) {
    frame.render_widget(
        Paragraph::new(format!("Counter: {}", model.counter)),
        frame.area(),
    );
}
```

The main loop then: draw the view, read an event, map it to a `Message`, call `update` (looping while it returns `Some`), repeat until `running_state == Done`. Note ratatui's docs deliberately *mutate* the model in place rather than returning a fresh one — idiomatic Rust over strict Elm immutability.

**Component Architecture (the official `component` template).** For large apps with many independent panels, a trait-based approach co-locates each panel's event handling, update, and rendering:

```rust
pub trait Component {
    fn init(&mut self) -> Result<()> { Ok(()) }
    fn handle_events(&mut self, event: Option<Event>) -> Action {
        match event {
            Some(Event::Key(k)) => self.handle_key_events(k),
            Some(Event::Mouse(m)) => self.handle_mouse_events(m),
            Some(Event::Tick) => Action::Tick,
            _ => Action::Noop,
        }
    }
    fn handle_key_events(&mut self, key: KeyEvent) -> Action { Action::Noop }
    fn handle_mouse_events(&mut self, mouse: MouseEvent) -> Action { Action::Noop }
    fn update(&mut self, action: Action) -> Action { Action::Noop }
    fn render(&mut self, f: &mut Frame, rect: Rect);
}
```

Each component owns its private state; the app holds a `Vec<Box<dyn Component>>` and dispatches actions to all of them. Contrast with TEA's single global model and single `update`. Projects like `gobang` and `edma` use this style. The tradeoff: components scale to many panels and localize complexity, but distributing one global `update` across many components can make cross-component logic (shared state) harder — you route it via the `Action` channel.

**Flux.** Ratatui also documents a Flux variant (Dispatcher → Store → Actions → Views) for unidirectional data flow; it's essentially TEA with an explicit dispatcher and store separation, useful if you like the Redux mental model. A real example is the `rust-chat-server` TUI.

**Component/widget composition with `StatefulWidget`.** The recommended compositional pattern is a single root widget (often `App`) passed to `frame.render_widget`, whose `render` calls child widgets on sub-`Rect`s from a `Layout`. For state that must survive between frames (selection, scroll offset), use `StatefulWidget`:

```rust
pub trait StatefulWidget {
    type State;
    fn render(self, area: Rect, buf: &mut Buffer, state: &mut Self::State);
}
```

The app owns the `State` (e.g. `ListState`), and passes it in via `frame.render_stateful_widget(list, area, &mut self.list_state)`. The useful rule of thumb from the docs: *if recreating the widget should reset its state, widget-owned state is fine; if the state must persist, keep it outside the widget and pass it in.* Since 0.26 you can also `impl Widget for &mut MyWidget` when a widget needs to mutate its own state during render (e.g. a frame counter). For heterogeneous collections of widgets, the `WidgetRef`/`StatefulWidgetRef` traits (0.26+, still gated behind `unstable-widget-ref`) let you store `Box<dyn WidgetRef>` and render by reference. Nested stateful widgets compose by having a parent `State` struct that contains child `State` structs.

**How notable apps organize state vs. rendering:**
- **bottom** — a central `App` struct holds all state (`app.data_collection`); raw collected data is converted lazily into display-ready `ConvertedData`/`canvas_data`, and a `canvas::Painter` owns all drawing (`painter.draw_data(terminal, app)`). Clean model→converter→painter separation.
- **gitui** — tab/component structure; git state lives behind the `asyncgit` crate, UI components render it.
- **yazi** — a client-server "Data Distribution Service"; UI plugins (Lua) render, async tasks mutate state.
- **atuin** — uses ratatui's inline viewport (its team implemented that feature upstream) for the Ctrl-R search UI.

### 2. Concurrency: separating the UI refresh from heavy model-update work

**The problem.** If you read input and do heavy work (data collection, network, file I/O) on the same thread as `terminal.draw`, the UI hangs. The universal solution: move blocking/slow work off the render thread and communicate over channels using a message enum.

**Option A — OS threads + channels (no async).** This is what bottom uses and what the ratatui FAQ explicitly endorses when "not much" of your app benefits from async — its verbatim guidance is *"If the answer is not much, maybe it is simpler to not use async and avoiding tokio."*

Concretely, **bottom's architecture is three OS threads plus `std::sync::mpsc`** (no tokio anywhere):

1. A **collection thread** (`create_collection_thread`, spawned with `std::thread::spawn`) loops: run `DataCollector::update_data()`, wrap the result as `BottomEvent::Update(Box::from(data))`, send it over `std::sync::mpsc`, reset its buffer, then interruptibly sleep for the update interval.
2. An **input thread** (`create_input_thread`) runs crossterm's `poll`/`read` and sends `BottomEvent::KeyInput`/`MouseInput` on the same sender (throttled ~20ms).
3. The **main thread** `recv`s `BottomEvent`s from the single receiver (classic multi-producer/single-consumer), updates `App`, and repaints.

bottom's actual event enum:

```rust
#[derive(Debug)]
pub enum BottomEvent<I, J> {
    KeyInput(I),
    MouseInput(J),
    Update(Box<data_harvester::Data>),
    Clean,
}
```

A **second control channel** sends commands *to* the collection thread (reconfigure, reset, change update rate) without stopping it:

```rust
#[derive(Debug)]
pub enum ThreadControlEvent {
    Reset,
    UpdateConfig(Box<app::AppConfigFields>),
    UpdateUsedWidgets(Box<UsedWidgets>),
    UpdateUpdateTime(u64),
}
```

The collection thread polls this with a non-blocking `control_receiver.try_recv()` each cycle, and its sleep is *interruptible* (originally a `Condvar::wait_timeout`, in current versions a `cancellation_token.sleep_with_cancellation(...)`) so shutdown or a reset (`Ctrl-r` from the UI) wakes it immediately instead of waiting a full interval. Interestingly, `DataCollector::update_data()` is an async fn but bottom drives it synchronously with `futures::executor::block_on(...)` — proving you can consume async libraries without a tokio runtime. bottom uses ratatui (imported under the alias `tui`, historically ~0.27, bumped to ~0.30.2 in recent releases) on a `CrosstermBackend<Stdout>`.

A minimal thread-based skeleton:

```rust
enum Event { Input(KeyEvent), Tick, Data(Vec<Metric>) }

fn spawn_input(tx: std::sync::mpsc::Sender<Event>) {
    std::thread::spawn(move || loop {
        if crossterm::event::poll(Duration::from_millis(100)).unwrap() {
            if let CtEvent::Key(k) = crossterm::event::read().unwrap() {
                if tx.send(Event::Input(k)).is_err() { break; }
            }
        }
    });
}

fn spawn_worker(tx: std::sync::mpsc::Sender<Event>) {
    std::thread::spawn(move || loop {
        let data = collect_expensive_metrics();     // blocking work, off the UI thread
        if tx.send(Event::Data(data)).is_err() { break; }
        std::thread::sleep(Duration::from_secs(1));
    });
}

fn main() -> Result<()> {
    let mut terminal = ratatui::init();
    let (tx, rx) = std::sync::mpsc::channel();
    spawn_input(tx.clone());
    spawn_worker(tx.clone());
    let mut app = App::default();
    while app.running {
        terminal.draw(|f| view(&app, f))?;
        match rx.recv()? {            // blocks until *some* event arrives
            Event::Input(k) => app.on_key(k),
            Event::Data(d)  => app.metrics = d,
            Event::Tick     => app.on_tick(),
        }
    }
    ratatui::restore();
    Ok(())
}
```

**Which channel crate for the thread model?**
- `std::sync::mpsc` — standard library, single-consumer, zero dependencies. Fine for the "all events funnel to one receiver" TUI pattern (what bottom uses).
- `crossbeam-channel` — faster under contention, multi-producer *and* multi-consumer, and crucially provides a `select!` macro to wait on multiple channels at once. This is what **gitui** uses to wait for results from its git worker thread pool.
- `flume` — a drop-in `std::sync::mpsc` replacement that is MPMC, has a `select`-like API, and supports *both* sync and async on the same channel (useful for hybrid designs). Its README claims it is *"Always faster than std::sync::mpsc and sometimes crossbeam-channel."*

**Option B — async runtime (tokio).** The ratatui async/component templates, atuin, and yazi use tokio. Here the render loop, input, ticks, and background tasks are all tokio tasks, multiplexed with `tokio::select!`. Input becomes async via crossterm's `EventStream` (requires the `event-stream` feature + `futures`):

```rust
pub async fn run(&mut self) -> Result<()> {
    let tick_rate = Duration::from_secs_f64(1.0 / self.tick_rate);
    let frame_rate = Duration::from_secs_f64(1.0 / self.frame_rate);
    let mut tick_interval = tokio::time::interval(tick_rate);
    let mut frame_interval = tokio::time::interval(frame_rate);
    let mut events = crossterm::event::EventStream::new();
    loop {
        tokio::select! {
            _ = self.cancel.cancelled() => break,
            _ = tick_interval.tick()  => self.tx.send(Event::Tick)?,
            _ = frame_interval.tick() => self.tx.send(Event::Render)?,
            Some(Ok(evt)) = events.next() => { /* map crossterm event -> Event */ }
        }
    }
    Ok(())
}
```

Actions/work are then dispatched over a `tokio::sync::mpsc::UnboundedSender<Action>`; a component can `tokio::spawn` a network request and send an `Action` back when it completes, which the main loop picks up on the next iteration.

**Tradeoff summary (to help you choose):**

| Dimension | OS threads + channels | tokio + async |
|---|---|---|
| Complexity | Lower; no runtime, no `.await` coloring, easier to reason about | Higher; async coloring, `Send`/`'static` bounds, runtime setup |
| Best when | A few long-lived background jobs (metrics, one worker pool) | Many concurrent I/O operations (dozens of network requests, streaming) |
| Blocking work | Natural — just call it on a thread | Needs `spawn_blocking` or it stalls the executor |
| Ecosystem | `crossbeam-channel`, `flume`, thread pools | Huge async I/O ecosystem (reqwest, sqlx, tonic) |
| Reference apps | bottom, gitui | atuin, yazi, ratatui async/component templates |
| Channels | `std::sync::mpsc`, `crossbeam`, `flume` | `tokio::sync::mpsc`, `flume` (async mode) |

**Recommendation for an undecided developer:** default to **threads + channels** unless you can name a concrete need for many-concurrent-I/O. bottom is a complex, high-refresh, real-world monitor built entirely without tokio — proof the thread model scales. Adopt tokio when your *domain* (not your UI) is async-heavy. Note you can also start with threads and later run a small tokio runtime for just the I/O layer (hybrid). Avoid mixing blocking `crossbeam`/`std` channel `recv()` calls inside async tasks — that blocks the executor.

**Decoupling render rate from update rate.** Both models benefit from separate "tick" (state update) and "render" (frame) cadences. The ratatui async template ships with a default of **1 tick per second and 60 frames per second** (its CLI exposes `--tick-rate <FLOAT>` "number of ticks per second [default: 1]" and `--frame-rate <FLOAT>` "number of frames per second [default: 60]"), sending `Event::Tick` and `Event::Render` on separate intervals so the UI only redraws when a `Render` event arrives — this bounds CPU usage. Frame-rate limiting matters: redrawing at uncontrolled speed while holding a key can spike CPU.

### 3. The main event loop

The canonical shape is **draw → handle events → update → repeat**, breaking when a quit condition is set. The single most important structural decision is to **read input on a separate thread/task** and funnel *all* event sources (keyboard, mouse, timer ticks, background-task completions, resize) into one channel or one `select!`. This avoids the anti-pattern of blocking the draw loop for e.g. 250ms polling for a keypress, which couples input latency to render rate and produces odd behavior (holding a key redraws faster).

Handling multiple sources:
- **Thread model:** every producer holds a `Sender<Event>` clone; the main loop calls `rx.recv()`. To wait on several distinct channels, use `crossbeam-channel`'s `select!`.
- **Async model:** `tokio::select!` over the event stream, tick interval, frame interval, and action receiver.

**Graceful shutdown, terminal restore, and panic handling.** A ratatui app runs in raw mode on the alternate screen; if it panics without restoring, the user's terminal is left broken. Modern ratatui makes this easy:

- `ratatui::init()` sets up the terminal *and installs a panic hook that restores it automatically* — this behavior was introduced in ratatui **0.28.1** ("the panic hook is automatically set up by the new `ratatui::init` function, so you no longer need to manually set up the panic hook"). `ratatui::restore()` tears it down. The all-in-one `ratatui::run(|terminal| {...})` (init + restore + panic hook) was added in **0.30.0**.
- For rich errors, integrate **color_eyre**: call `color_eyre::install()?` in `main`, return `color_eyre::Result<()>`, and use `.wrap_err(...)` to add context. The panic/error hook must restore the terminal *before* printing, or the report is scrambled by raw mode.
- If you manage the terminal manually, install a hook that saves the original, restores the terminal (leave alternate screen, disable raw mode, disable mouse capture, show cursor), then calls the original hook:

```rust
pub fn install_panic_hook() {
    let original = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        let _ = crossterm::execute!(std::io::stdout(), LeaveAlternateScreen, DisableMouseCapture);
        let _ = crossterm::terminal::disable_raw_mode();
        original(info);
    }));
}
```

In the thread/async models, also signal worker threads to stop (a `CancellationToken` in tokio, or a shared flag / dropping the `Sender` in the thread model — bottom uses an interruptible condvar/cancellation token so the collection thread exits promptly). bottom additionally registers a `ctrlc` handler and a custom panic hook that calls `cleanup_terminal`.

### 4. Scalability & maintainability

**Project/module structure.** The official `component` template's layout is a good large-app baseline:

```
src/
├── main.rs        // tokio entry point: init errors+logging, parse CLI, App::new().run()
├── app.rs         // App struct: owns components, runs the event->action->draw loop
├── tui.rs         // Terminal + EventHandler wrapper (enter/exit, event stream, tick/render)
├── action.rs      // Action enum (messages)
├── components.rs  // Component trait
├── components/
│   ├── home.rs
│   └── fps.rs
├── config.rs      // config + keybindings
├── cli.rs         // clap args (tick-rate, frame-rate)
├── errors.rs      // color_eyre / panic hooks
└── logging.rs     // tracing
```

The template deliberately isolates ratatui-specific plumbing (`tui.rs`) from business logic (`components/`), so "you shouldn't need to change anything outside the `components` folder." Add supporting concerns most real TUIs need: CLI parsing (clap), XDG config directories, logging (tracing), and the panic handler.

**Focus, routing input, and multiple screens/tabs.** Keep a notion of "current focus" (an enum of which panel/tab is active) in the model, and route key events to the focused component — the same physical key means different things depending on focus (typing "j" into a text field vs. scrolling a list). For multi-screen apps, model screens/tabs as an enum and match on it in both `update` and `view`; the `Tabs` widget renders the tab bar. The component architecture handles this by only having the focused component consume the event.

**Shared state / avoiding prop-drilling.** Options, roughly in order of preference: (1) a central model that the view reads (TEA) — simplest; (2) pass a `&mut AppContext` "context object" holding shared data down through calls (a pattern the ratatui maintainers note yazi historically used heavily); (3) an action/message channel so components communicate without holding references to each other (the component template stores an `action_tx` in each component); (4) actor-like patterns (each subsystem owns its state, communicates by messages) for the most decoupled designs. Reach for `Arc<Mutex<...>>` only when genuinely sharing mutable state across threads/tasks, and keep lock scopes tiny.

**Testing.** Ratatui has first-class test support:
- **`TestBackend`** renders to an in-memory `Buffer` you can assert against with `assert_buffer_lines` / `assert_eq`:

```rust
#[test]
fn renders_counter() {
    let mut terminal = Terminal::new(TestBackend::new(20, 3)).unwrap();
    let app = App::default();
    terminal.draw(|f| f.render_widget(&app, f.area())).unwrap();
    assert_eq!(terminal.backend().buffer().cell((0,0)).unwrap().symbol(), "C");
}
```

- **`insta` snapshot tests** capture the whole rendered screen as text; `cargo insta review` accepts intentional changes. Use a fixed terminal size (e.g. 80×20) for reproducibility. Caveat: color assertions aren't supported by `TestBackend`'s display yet.
- The ratatui maintainers advise preferring unit tests directly against the `Buffer`/widget over full `TestBackend` integration tests where possible, and testing pure `update` logic directly (a major reason TEA is praised for testability — `update` is a pure-ish function of `(state, message)`).

**Error handling.** Use `color_eyre::Result<T>` (or `anyhow`) at the app boundary and `?` throughout; add context with `.wrap_err(...)`. Keep error messages lowercase, concise, no trailing punctuation (idiomatic Rust). Make sure any error bubbling to `main` still restores the terminal (bottom explicitly fixed a bug where an `Err` bubbling to the top failed to clean up the terminal).

### 5. Official & community resources
- **Official docs (ratatui.rs):** Application Patterns (TEA / Component / Flux), Concepts → Widgets & Rendering, the async counter-app tutorial (Async Event Stream, Full Async Events, Full Async Actions), Recipes → Testing (TestBackend, insta snapshots), Recipes → Apps (panic hooks, color_eyre, logging with tracing, CLI args, config directories). FAQ has the key async-vs-threads guidance.
- **Templates (github.com/ratatui/templates):** `simple`, `simple-async`, and `component` (async tokio + crossterm, with `Component` trait, tick/frame rates, tracing, color_eyre, clap). The older `ratatui/async-template` is archived → use `templates`.
- **docs.rs/ratatui:** `Widget`, `StatefulWidget`, `StatefulWidgetRef`, `TestBackend`, `Terminal`, `ratatui::run`/`init`/`restore`.
- **awesome-ratatui** lists widget crates and example apps; **tui-realm** (React/Elm-style framework) and **tears**/**ratatui-elm** (TEA frameworks) exist if you want batteries included.
- **Reference codebases:** bottom (ClementTsang/bottom — threads + mpsc, ratatui via `tui` alias), gitui (crossbeam-channel worker pool via the `asyncgit` crate), atuin (tokio, inline viewport), yazi (fully async tokio task scheduler).

## Recommendations
1. **Start from the official `component` template** (`cargo generate ratatui/templates component`) if you expect multiple panels/tabs, or hand-roll a TEA loop if the app is essentially one screen with one state machine. Both are legitimate; the template gives you logging, config, CLI, and panic handling for free.
2. **Choose concurrency by workload, not fashion.** Default to **OS threads + `std::sync::mpsc`** (or `crossbeam-channel` if you need `select!`/MPMC, as gitui does). Switch to **tokio** only when you have concrete many-concurrent-I/O needs (many network calls, streaming), as atuin/yazi do. Threshold to change your mind: if you find yourself spawning more than a handful of concurrent I/O jobs or hand-rolling a futures executor, move that layer to tokio.
3. **Always funnel every event source into one channel/`select!`, read input off the render thread, and put all state mutation in one `update`/action handler.** This single decision prevents the most common TUI bugs (input lag, inconsistent state, UI hangs).
4. **Decouple tick rate from frame rate** (e.g. tick ≈ your data cadence, frame ≤ 60fps) and only redraw on render events to keep CPU low.
5. **Install `ratatui::init()`/`restore()` + `color_eyre::install()` on day one**, and verify the terminal restores on both panic and `Err`-to-main.
6. **Write `TestBackend`/`insta` tests for your views and plain unit tests for your `update` logic** as you build, not after; keep `view` pure so it's trivially testable.
7. **Use `StatefulWidget` + app-owned `ListState`/`TableState`** for any scrollable/selectable list; don't store selection inside a widget that's recreated each frame.

## Caveats
- The ratatui maintainers explicitly call the website's TEA article "pedagogical" / "fan fiction with regard to [strict] TEA" — the patterns are sound idioms but not a rigid framework; adapt them.
- bottom's internal type/module names have shifted across versions (`data_harvester` → `collection`, `ThreadControlEvent` → `CollectionThreadEvent`, condvar → cancellation token; ratatui bumped from ~0.27 to ~0.30.2). The *architecture* (three OS threads + `std::sync::mpsc` + a `BottomEvent`-style enum + a control channel) is stable, but exact identifiers depend on the release you read.
- The `WidgetRef`/`StatefulWidgetRef` traits remain gated behind the `unstable-widget-ref` feature; APIs may change.
- `TestBackend` cannot assert colors/styles via its text output yet — snapshot/style testing is limited to the character grid.
- Version churn is real: **ratatui 0.30.0 reorganized the project into a modular workspace** (`ratatui-core` for widget-library authors, `ratatui-widgets`, backend crates), but *"Main crate (recommended for apps)"* remains the umbrella `ratatui` crate — most apps should keep depending on it. The latest release line is **0.30.2** (MSRV 1.88.0). Crossterm compatibility is handled via feature flags (`crossterm_0_28` / `crossterm_0_29`), with `crossterm_0_29` now the default in ratatui 0.30; async examples pinning `crossterm 0.28` may need to align their crossterm version with your ratatui version to avoid the "two crossterm major versions" type errors.