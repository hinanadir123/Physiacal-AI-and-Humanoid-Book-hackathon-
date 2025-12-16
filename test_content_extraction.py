import requests
from bs4 import BeautifulSoup
from bs4 import Comment

def extract_text_from_url(url: str) -> str:
    """
    Extracts clean text from a given URL, excluding navigation, footer, and code blocks.
    """
    try:
        response = requests.get(url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, "html.parser")

        # Remove navigation elements
        for nav in soup.find_all(['nav', 'header', 'footer']):
            nav.decompose()

        # Remove sidebar navigation
        for sidebar in soup.find_all(class_=['menu', 'sidebar', 'navigation']):
            sidebar.decompose()

        # Remove code blocks and preformatted text
        for code_elem in soup.find_all(['pre', 'code', 'kbd', 'samp']):
            code_elem.decompose()

        # Remove script and style elements
        for script in soup.find_all(['script', 'style']):
            script.decompose()

        # Remove comments
        for comment in soup.find_all(string=lambda text: isinstance(text, Comment)):
            comment.extract()

        # Docusaurus typically wraps main content in an <article> tag
        article = soup.find("article")
        if article:
            # Remove any remaining unwanted elements within the article
            for unwanted in article.find_all(['nav', 'header', 'footer', 'aside']):
                unwanted.decompose()
            return article.get_text(separator="\n", strip=True)

        # Look for main content containers with common Docusaurus class names
        main_content = soup.find("main") or soup.find(class_="main") or soup.find(id="main")
        if main_content:
            # Remove any remaining unwanted elements within the main content
            for unwanted in main_content.find_all(['nav', 'header', 'footer', 'aside']):
                unwanted.decompose()
            return main_content.get_text(separator="\n", strip=True)

        # Look for content containers
        content = soup.find(class_="container") or soup.find(class_="content") or soup.find(class_="doc-content")
        if content:
            for unwanted in content.find_all(['nav', 'header', 'footer', 'aside']):
                unwanted.decompose()
            return content.get_text(separator="\n", strip=True)

        # Fallback to body content
        for unwanted in soup.body.find_all(['nav', 'header', 'footer', 'aside']):
            unwanted.decompose()
        return soup.body.get_text(separator="\n", strip=True)
    except requests.exceptions.RequestException as e:
        print(f"Error fetching URL {url}: {e}")
        return ""

if __name__ == "__main__":
    # Test with a few URLs from the site
    test_urls = [
        "https://physiacal-ai-and-humanoid-book-hack.vercel.app/",
        "https://physiacal-ai-and-humanoid-book-hack.vercel.app/docs/intro",
        "https://physiacal-ai-and-humanoid-book-hack.vercel.app/blog"
    ]

    for url in test_urls:
        print(f"\nTesting URL: {url}")
        content = extract_text_from_url(url)
        print(f"Content length: {len(content)} characters")
        print(f"Content preview: {content[:200] if content else 'No content'}...")