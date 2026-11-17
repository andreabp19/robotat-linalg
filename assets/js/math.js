// For formatting LaTex equations
document.addEventListener('DOMContentLoaded', () => {
  renderMathInElement(document.querySelector('.guide-content'), {
    delimiters: [
      {left: '$$', right: '$$', display: true},
      {left: '\\[', right: '\\]', display: true},
      {left: '$',  right: '$',  display: false},
      {left: '\\(', right: '\\)', display: false}
    ]
  });
});