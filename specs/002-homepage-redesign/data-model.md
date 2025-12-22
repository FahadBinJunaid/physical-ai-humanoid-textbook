# Data Model: Homepage Redesign

## Module Card Entity

**Entity Name**: ModuleCard
- **title**: string - The display title of the learning module (e.g., "ROS2 Basics", "Digital Twin", "NVIDIA Isaac", "VLA Intelligence")
- **description**: string - A brief description of the module content (optional, for future enhancement)
- **link**: string - The URL path to the module's documentation
- **icon**: string - An optional icon identifier for visual representation
- **color**: string - Theme color for the card (for visual consistency)

## Navigation Element Entity

**Entity Name**: NavigationElement
- **label**: string - The display text for the navigation item
- **path**: string - The URL path for the navigation item
- **type**: string - The type of navigation (e.g., "footer", "navbar", "module-card")
- **priority**: number - The order in which the element should appear

## Theme Configuration Entity

**Entity Name**: ThemeConfig
- **name**: string - The theme identifier (e.g., "light", "dark")
- **primaryColor**: string - The primary color for the theme
- **secondaryColor**: string - The secondary color for the theme
- **textColor**: string - The color for text elements
- **backgroundColor**: string - The background color
- **contrastRatio**: number - The minimum contrast ratio for accessibility

## Branding Configuration Entity

**Entity Name**: BrandingConfig
- **title**: string - The site title ("Physical AI & Humanoid Robotics")
- **tagline**: string - The site tagline ("Mastering the Future of Embodied Intelligence")
- **logoPath**: string - Path to the site logo (if applicable)
- **faviconPath**: string - Path to the site favicon (if applicable)