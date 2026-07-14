
(function () {
  const slides = document.querySelectorAll('#benchmarks .slide');
  const dotsContainer = document.getElementById('benchmarkDots');
  const captionEl = document.getElementById('benchmarkCaption');
  let current = 0;

  slides.forEach((_, i) => {
    const dot = document.createElement('button');
    dot.className = 'dot' + (i === 0 ? ' active' : '');
    dot.setAttribute('aria-label', 'Go to benchmark ' + (i + 1));
    dot.addEventListener('click', () => goTo(i));
    dotsContainer.appendChild(dot);
  });
  const dots = dotsContainer.querySelectorAll('.dot');

  function goTo(index) {
    slides[current].classList.remove('active');
    dots[current].classList.remove('active');
    current = (index + slides.length) % slides.length;
    slides[current].classList.add('active');
    dots[current].classList.add('active');
    captionEl.textContent = slides[current].dataset.caption;
  }

  document.querySelector('#benchmarks .prev').addEventListener('click', () => goTo(current - 1));
  document.querySelector('#benchmarks .next').addEventListener('click', () => goTo(current + 1));

  goTo(0); // initialize
})();