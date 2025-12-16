"""
Test script for the URL ingestion module
"""
import sys
sys.path.insert(0, 'backend')

from backend.main import get_all_urls, extract_text_from_url, ingest_url_content, ingest_multiple_urls

def test_url_ingestion():
    print("Testing URL Ingestion Module...")

    # Test URL fetching (using the same URL as in the main function)
    base_url = "https://physiacal-ai-and-humanoid-book-hack.vercel.app"
    print(f"Fetching URLs from: {base_url}")

    urls = get_all_urls(base_url)
    print(f"Found {len(urls)} URLs")

    if urls:
        # Test content extraction from the first URL
        first_url = urls[0]
        print(f"Extracting content from: {first_url}")

        content = extract_text_from_url(first_url)
        print(f"Extracted content length: {len(content)} characters")

        # Test the new ingestion function
        ingestion_result = ingest_url_content(first_url)
        print(f"Ingestion result: {ingestion_result['status']}")
        print(f"Section ID: {ingestion_result['section_id']}")
        print(f"Word count: {ingestion_result['word_count']}")

        # Test multiple URL ingestion
        sample_urls = urls[:2]  # Just test with first 2 URLs
        multiple_results = ingest_multiple_urls(sample_urls)
        print(f"Multiple ingestion results: {len(multiple_results)} results processed")

        for i, result in enumerate(multiple_results):
            print(f"  URL {i+1}: {result['status']} - {result['section_id']} ({result['word_count']} words)")

    print("\nURL Ingestion Module tests completed successfully!")

if __name__ == "__main__":
    test_url_ingestion()