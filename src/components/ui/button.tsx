import { cva, type VariantProps } from "class-variance-authority";
import { type ButtonHTMLAttributes, forwardRef } from "react";
import { cn } from "@/lib/utils";

const buttonVariants = cva(
  "inline-flex items-center justify-center gap-2 font-medium transition-colors duration-150 ease-[cubic-bezier(0.22,1,0.36,1)] focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring/70 disabled:pointer-events-none disabled:opacity-40 active:scale-[0.98]",
  {
    variants: {
      variant: {
        default: "bg-accent text-accent-foreground hover:bg-accent/90",
        secondary: "bg-card-2 text-foreground hover:bg-border",
        outline: "border border-border bg-transparent hover:bg-card-2",
        ghost: "hover:bg-card-2",
        stop: "bg-stop text-foreground hover:bg-stop/90",
        go: "bg-go text-background hover:bg-go/90",
        warn: "bg-warn text-background hover:bg-warn/90",
      },
      size: {
        default: "h-10 px-4 text-sm rounded-md",
        sm: "h-8 px-3 text-xs rounded-sm",
        lg: "h-12 px-5 text-sm rounded-md",
        icon: "size-10 rounded-md",
      },
    },
    defaultVariants: { variant: "default", size: "default" },
  },
);

export const Button = forwardRef<
  HTMLButtonElement,
  ButtonHTMLAttributes<HTMLButtonElement> & VariantProps<typeof buttonVariants>
>(({ className, variant, size, ...props }, ref) => (
  <button ref={ref} className={cn(buttonVariants({ variant, size }), className)} {...props} />
));
Button.displayName = "Button";
