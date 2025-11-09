"""
Interactive example runner for Vuer scenes.

This script provides an interactive menu to select and run scene examples.

Usage:
    python -m vuer_mjcf.vuer_examples.run_example
    python -m vuer_mjcf.vuer_examples.run_example <example_name>
"""

import sys
from vuer_mjcf.vuer_examples.list_examples import EXAMPLES, get_example_by_name


def run_example(example_name):
    """Run a specific example by name."""
    example = get_example_by_name(example_name)

    if not example:
        print(f"Error: Example '{example_name}' not found.")
        print("\nAvailable examples:")
        for category, examples in EXAMPLES.items():
            print(f"\n{category}:")
            for ex in examples:
                print(f"  - {ex['name']}")
        return False

    print(f"\n{'=' * 60}")
    print(f"Running: {example['name']}")
    print(f"Description: {example['description']}")
    print(f"Complexity: {example['complexity']}")
    print(f"{'=' * 60}\n")

    # Import and run the example
    module_path = example['module']
    try:
        import importlib
        module = importlib.import_module(module_path)
        module.main()
        return True
    except Exception as e:
        print(f"Error running example: {e}")
        import traceback
        traceback.print_exc()
        return False


def interactive_menu():
    """Display an interactive menu to select examples."""
    print("\n" + "=" * 60)
    print("VUER EXAMPLES - Interactive Runner")
    print("=" * 60)

    # Build list of all examples with numbers
    all_examples = []
    for category, examples in EXAMPLES.items():
        all_examples.extend(examples)

    # Display menu
    print("\nSelect an example to run:\n")

    idx = 1
    for category, examples in EXAMPLES.items():
        print(f"\n{category}")
        print("-" * 60)
        for ex in examples:
            print(f"  [{idx}] {ex['complexity']} {ex['name']}")
            print(f"      {ex['description']}")
            idx += 1

    print(f"\n  [0] Exit")
    print("\n" + "=" * 60)

    # Get user selection
    try:
        choice = input("\nEnter number (0 to exit): ").strip()
        choice = int(choice)

        if choice == 0:
            print("Exiting...")
            return

        if choice < 1 or choice > len(all_examples):
            print(f"Invalid choice. Please enter a number between 0 and {len(all_examples)}")
            return

        # Run selected example
        selected = all_examples[choice - 1]
        run_example(selected['name'])

    except ValueError:
        print("Invalid input. Please enter a number.")
    except KeyboardInterrupt:
        print("\n\nExiting...")
    except Exception as e:
        print(f"\nError: {e}")


def main():
    """Main entry point."""
    if len(sys.argv) > 1:
        # Run specific example from command line
        example_name = sys.argv[1]
        run_example(example_name)
    else:
        # Show interactive menu
        interactive_menu()


if __name__ == "__main__":
    main()
