document.addEventListener('DOMContentLoaded', () => {
    const searchInput = document.getElementById('search-input');
    const searchType = document.getElementById('search-type');
    const searchButton = document.getElementById('search-button');
    const resultsContainer = document.getElementById('results-container');
    const popularTermsList = document.getElementById('popular-terms-list');

    const performSearch = async () => {
        const term = searchInput.value.trim();
        const type = searchType.value;

        if (term.length < 1) {
            resultsContainer.innerHTML = '<p class="placeholder-text">请输入关键词进行搜索</p>';
            return;
        }

        resultsContainer.innerHTML = '<p class="placeholder-text">正在搜索...</p>';

        try {
            const response = await fetch(`/api/search?term=${encodeURIComponent(term)}&type=${encodeURIComponent(type)}`);
            if (!response.ok) {
                throw new Error('网络响应错误');
            }
            const results = await response.json();
            displayResults(results);
        } catch (error) {
            console.error('搜索失败:', error);
            resultsContainer.innerHTML = '<p class="placeholder-text">搜索时发生错误，请稍后重试。</p>';
        }
    };

    const displayResults = (results) => {
        if (results.length === 0) {
            resultsContainer.innerHTML = '<p class="placeholder-text">未找到相关结果</p>';
            return;
        }

        let html = '';
        results.forEach(item => {
            html += `
                <div class="result-item">
                    <div class="result-header">
                        <span class="result-title">${item.result_title || '无标题'}</span>
                        <span class="result-type">${item.result_type}</span>
                    </div>
                    <p class="result-description">${item.result_description || ''}</p>
                </div>
            `;
        });
        resultsContainer.innerHTML = html;
    };

    const fetchAndDisplayPopularTerms = async () => {
        try {
            const response = await fetch('/api/popular_terms');
            if (!response.ok) {
                throw new Error('网络响应错误');
            }
            const terms = await response.json();
            
            let html = '';
            terms.forEach(term => {
                html += `<span class="popular-term" data-term="${term.search_term}">${term.search_term}</span>`;
            });
            popularTermsList.innerHTML = html;

            // Add click event listeners to popular terms
            document.querySelectorAll('.popular-term').forEach(el => {
                el.addEventListener('click', () => {
                    searchInput.value = el.dataset.term;
                    performSearch();
                });
            });

        } catch (error) {
            console.error('获取热门搜索词失败:', error);
        }
    };

    searchButton.addEventListener('click', performSearch);
    searchInput.addEventListener('keyup', (event) => {
        if (event.key === 'Enter') {
            performSearch();
        }
    });
    
    // Initial load
    fetchAndDisplayPopularTerms();
}); 