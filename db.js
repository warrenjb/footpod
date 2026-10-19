// Shared Supabase code for the Foot Pod pages: sign-in, who's working out,
// and an offline-safe upload queue. Same project and login as the sit-up counter.
const FootDB = (() => {
  const SUPABASE_URL = 'https://jhunufgtetzqkswtstdv.supabase.co';
  const SUPABASE_KEY = 'sb_publishable_iWbHvHXWYXeVqFmxJNu_vw_zvvmYj_J'; // publishable: safe in a page
  const QUEUE_KEY = 'footpod_pending_rows';
  const PERSON_KEY = 'footpod_person';
  const PEOPLE = ['Warren', 'Other'];

  const sb = window.supabase ? window.supabase.createClient(SUPABASE_URL, SUPABASE_KEY) : null;
  let user = null, flushing = false;
  const authListeners = [];

  const uuid = () => (crypto.randomUUID ? crypto.randomUUID() :
    'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, c => {
      const r = Math.random() * 16 | 0;
      return (c === 'x' ? r : (r & 0x3 | 0x8)).toString(16);
    }));

  function load(key, fallback) {
    try { const v = localStorage.getItem(key); return v === null ? fallback : JSON.parse(v); } catch (_) { return fallback; }
  }
  function store(key, value) { try { localStorage.setItem(key, JSON.stringify(value)); } catch (_) {} }

  let queue = load(QUEUE_KEY, []);

  const syncState = {};          // client_id -> [text, kind], so redrawn lists keep their status
  function markSync(id, text, kind = '') {
    syncState[id] = [text, kind];
    document.querySelectorAll(`.sync[data-id="${id}"]`).forEach(el => {
      el.textContent = text;
      el.className = 'sync ' + kind;
    });
  }

  function enqueue(table, row) {
    queue.push({ table, row });
    store(QUEUE_KEY, queue);
    markSync(row.client_id, 'Saving…');
    flush();
  }

  async function flush() {
    if (!sb || flushing) return;
    if (!user) {
      queue.forEach(q => markSync(q.row.client_id, 'Not saved yet: sign in below', 'bad'));
      return;
    }
    flushing = true;
    try {
      while (queue.length) {
        const { table, row } = queue[0];
        const { error } = await sb.from(table).insert(row);
        if (error && error.code !== '23505') {             // 23505 = already saved
          markSync(row.client_id, 'Not saved: will retry', 'bad');
          console.warn('Save failed', error);
          break;
        }
        queue.shift();
        store(QUEUE_KEY, queue);
        markSync(row.client_id, 'Saved', 'ok');
      }
    } finally {
      flushing = false;
    }
    if (queue.length) setTimeout(flush, 30000);
  }

  // Sign-in card. `noun` is what gets saved ("runs", "sets").
  function accountUI(container, noun = 'workouts') {
    container.innerHTML = `
      <div class="acct-row fp-in" style="display:none">
        <span class="who">Saving ${noun} as <b class="fp-email"></b></span>
        <button class="small fp-out">Sign out</button>
      </div>
      <form class="acct-form fp-form">
        <h2>Sign in to save your ${noun}</h2>
        <input type="email" placeholder="Email" autocomplete="username" required>
        <input type="password" placeholder="Password" autocomplete="current-password" required>
        <button type="submit" class="primary">Sign in</button>
        <p class="note" style="margin:0">Same login as the sit-up counter.</p>
      </form>
      <div class="acct-msg"></div>`;
    const inRow = container.querySelector('.fp-in'), form = container.querySelector('.fp-form');
    const msg = container.querySelector('.acct-msg');
    const setMsg = (t, bad) => { msg.textContent = t; msg.className = 'acct-msg' + (bad ? ' bad' : ''); };
    const show = () => {
      inRow.style.display = user ? 'flex' : 'none';
      form.style.display = user ? 'none' : 'grid';
      container.querySelector('.fp-email').textContent = user ? user.email : '';
      if (user && queue.length) setMsg(`${queue.length} item(s) waiting to upload…`);
      else if (user) setMsg('');
    };
    if (!sb) { setMsg('Could not load the Supabase library. Nothing will be saved online.', true); form.style.display = 'none'; return; }
    authListeners.push(show);
    show();
    form.addEventListener('submit', async e => {
      e.preventDefault();
      setMsg('Signing in…');
      const [email, pw] = form.querySelectorAll('input');
      const { error } = await sb.auth.signInWithPassword({ email: email.value.trim(), password: pw.value });
      if (error) setMsg(error.message, true); else { setMsg(''); pw.value = ''; }
    });
    container.querySelector('.fp-out').onclick = () => sb.auth.signOut();
  }

  function onAuth(fn) { authListeners.push(fn); if (sb) fn(user); }

  if (sb) {
    sb.auth.onAuthStateChange((_event, session) => {
      user = session ? session.user : null;
      authListeners.forEach(fn => fn(user));
      if (user) flush();
    });
    window.addEventListener('online', flush);
  }

  // "Who's working out" picker, remembered across pages.
  function personUI(select) {
    select.innerHTML = PEOPLE.map(p => `<option>${p}</option>`).join('');
    const saved = load(PERSON_KEY, null);
    if (saved && PEOPLE.includes(saved)) select.value = saved;
    select.addEventListener('change', () => store(PERSON_KEY, select.value));
    return () => select.value;
  }

  // HTML for a status badge that reflects the latest known state of `id`.
  function syncBadge(id) {
    const [text, kind] = syncState[id] || ['Saving…', ''];
    return `<span class="sync ${kind}" data-id="${id}">${text}</span>`;
  }

  return { sb, uuid, enqueue, flush, accountUI, personUI, onAuth, markSync, syncBadge, load, store, PEOPLE,
           get user() { return user; } };
})();
