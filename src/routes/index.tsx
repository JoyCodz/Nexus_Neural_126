import { createFileRoute } from "@tanstack/react-router";
import { OperatorConsole } from "@/components/vista/console";

export const Route = createFileRoute("/")({
  component: Home,
});

function Home() {
  return <OperatorConsole />;
}
