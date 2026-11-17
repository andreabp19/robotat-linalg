document.addEventListener('DOMContentLoaded', () => {
  let here = location.pathname.split('/').pop() || 'index.html';
  if (!here.includes('.')) here += '.html';

  document.querySelectorAll('.guide-sidebar a').forEach(a => {
    const href = a.getAttribute('href');
    const page = href.split('#')[0];
    if (page !== here) return;

    if (!href.includes('#')) a.setAttribute('aria-current', 'page');

    for (let d = a.closest('details'); d; d = d.parentElement.closest('details')) {
      d.open = true;
    }
  });

  function markHash() {
    document.querySelectorAll('.guide-sidebar a[href*="#"]').forEach(a => {
      const [page, hash] = a.getAttribute('href').split('#');
      a.classList.toggle('active', page === here && '#' + hash === location.hash);
    });
  }
  markHash();
  window.addEventListener('hashchange', markHash);
});